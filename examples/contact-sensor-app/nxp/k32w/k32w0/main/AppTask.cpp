/*
 *
 *    Copyright (c) 2021 Project CHIP Authors
 *    Copyright (c) 2021 Google LLC.
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */
#include "AppTask.h"
#include "AppEvent.h"
#include <app/server/Server.h>
#include <lib/support/ErrorStr.h>

#include <app/server/OnboardingCodesUtil.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <lib/support/ThreadOperationalDataset.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/internal/DeviceNetworkInfo.h>

#include <app-common/zap-generated/attribute-id.h>
#include <app-common/zap-generated/attribute-type.h>
#include <app-common/zap-generated/cluster-id.h>
#include <app/util/attribute-storage.h>

/* OTA related includes */
#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
#include "OTAImageProcessorImpl.h"
#include "OtaSupport.h"
#include <app/clusters/ota-requestor/BDXDownloader.h>
#include <app/clusters/ota-requestor/DefaultOTARequestor.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorDriver.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorStorage.h>
#endif

#include "Keyboard.h"
#include "LED.h"
#include "LEDWidget.h"
#include "app_config.h"

#if CHIP_CRYPTO_HSM
#include <crypto/hsm/CHIPCryptoPALHsm.h>
#endif
#ifdef ENABLE_HSM_DEVICE_ATTESTATION
#include "DeviceAttestationSe05xCredsExample.h"
#endif

#define FACTORY_RESET_TRIGGER_TIMEOUT 6000
#define APP_EVENT_QUEUE_SIZE 10

TimerHandle_t sFunctionTimer; // FreeRTOS app sw timer.

static QueueHandle_t sAppEventQueue;

#if !cPWR_UsePowerDownMode
static LEDWidget sStatusLED;
static LEDWidget sContactSensorLED;
#endif

static bool sIsThreadProvisioned = false;
static bool sHaveBLEConnections  = false;

static uint32_t eventMask = 0;

#if CHIP_DEVICE_CONFIG_THREAD_ENABLE_CLI
extern "C" void K32WUartProcess(void);
#endif

using namespace ::chip::Credentials;
using namespace ::chip::DeviceLayer;
using namespace chip;
;

AppTask AppTask::sAppTask;

#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
static DefaultOTARequestor gRequestorCore;
static DefaultOTARequestorStorage gRequestorStorage;
static DeviceLayer::DefaultOTARequestorDriver gRequestorUser;
static BDXDownloader gDownloader;
static OTAImageProcessorImpl gImageProcessor;

constexpr uint16_t requestedOtaBlockSize = 1024;
#endif

CHIP_ERROR AppTask::StartAppTask()
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    sAppEventQueue = xQueueCreate(APP_EVENT_QUEUE_SIZE, sizeof(AppEvent));
    if (sAppEventQueue == NULL)
    {
        err = APP_ERROR_EVENT_QUEUE_FAILED;
        K32W_LOG("Failed to allocate app event queue");
        assert(err == CHIP_NO_ERROR);
    }

    return err;
}

CHIP_ERROR AppTask::Init()
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    // Init ZCL Data Model and start server
    PlatformMgr().ScheduleWork(InitServer, 0);

    // Initialize device attestation config
#ifdef ENABLE_HSM_DEVICE_ATTESTATION
    SetDeviceAttestationCredentialsProvider(Examples::GetExampleSe05xDACProvider());
#else
    SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
    PlatformMgr().ScheduleWork(InitOTA, 0);
#endif

    // QR code will be used with CHIP Tool
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

    /* HW init leds */
#if !cPWR_UsePowerDownMode
    LED_Init();

    /* start with all LEDS turnedd off */
    sStatusLED.Init(SYSTEM_STATE_LED);

    sContactSensorLED.Init(CONTACT_SENSOR_STATE_LED);
    sContactSensorLED.Set(!ContactSensorMgr().IsUnlocked());
#endif
    UpdateClusterState();

    /* intialize the Keyboard and button press calback */
    KBD_Init(KBD_Callback);

    // Create FreeRTOS sw timer for Function Selection.
    sFunctionTimer = xTimerCreate("FnTmr",          // Just a text name, not used by the RTOS kernel
                                  1,                // == default timer period (mS)
                                  false,            // no timer reload (==one-shot)
                                  (void *) this,    // init timer id = app task obj context
                                  TimerEventHandler // timer callback handler
    );
    if (sFunctionTimer == NULL)
    {
        err = APP_ERROR_CREATE_TIMER_FAILED;
        K32W_LOG("app_timer_create() failed");
        assert(err == CHIP_NO_ERROR);
    }

    if (ContactSensorMgr().Init() != 0)
    {
        K32W_LOG("ContactSensorMgr().Init() failed");
        assert(status == 0);
    }

    ContactSensorMgr().SetCallbacks(ActionInitiated, ActionCompleted);


    // Print the current software version
    char currentSoftwareVer[ConfigurationManager::kMaxSoftwareVersionStringLength + 1] = { 0 };
    err = ConfigurationMgr().GetSoftwareVersionString(currentSoftwareVer, sizeof(currentSoftwareVer));
    if (err != CHIP_NO_ERROR)
    {
        K32W_LOG("Get version error");
        assert(err == CHIP_NO_ERROR);
    }

    K32W_LOG("Current Software Version: %s", currentSoftwareVer);

    PlatformMgr().AddEventHandler(ThreadProvisioningHandler, 0);

    return err;
}

void AppTask::InitServer(intptr_t arg)
{
    static chip::CommonCaseDeviceServerInitParams initParams;
    (void) initParams.InitializeStaticResourcesBeforeServerInit();

    // Init ZCL Data Model and start server
    VerifyOrDie((chip::Server::GetInstance().Init(initParams)) == CHIP_NO_ERROR);
}

#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
void AppTask::InitOTA(intptr_t arg)
{
    // Initialize and interconnect the Requestor and Image Processor objects -- START
    SetRequestorInstance(&gRequestorCore);

    gRequestorStorage.Init(chip::Server::GetInstance().GetPersistentStorage());
    gRequestorCore.Init(chip::Server::GetInstance(), gRequestorStorage, gRequestorUser, gDownloader);
    gRequestorUser.SetMaxDownloadBlockSize(requestedOtaBlockSize);
    gRequestorUser.Init(&gRequestorCore, &gImageProcessor);
    gImageProcessor.SetOTADownloader(&gDownloader);

    // Connect the gDownloader and Image Processor objects
    gDownloader.SetImageProcessorDelegate(&gImageProcessor);
    // Initialize and interconnect the Requestor and Image Processor objects -- END
}
#endif

void AppTask::AppTaskMain(void * pvParameter)
{
    AppEvent event;

    CHIP_ERROR err = sAppTask.Init();
    if (err != CHIP_NO_ERROR)
    {
        K32W_LOG("AppTask.Init() failed");
        assert(err == CHIP_NO_ERROR);
    }

    while (true)
    {
        TickType_t xTicksToWait = pdMS_TO_TICKS(10);

#if defined(cPWR_UsePowerDownMode) && (cPWR_UsePowerDownMode)
        xTicksToWait = portMAX_DELAY;
#endif

        BaseType_t eventReceived = xQueueReceive(sAppEventQueue, &event, xTicksToWait);
        while (eventReceived == pdTRUE)
        {
            sAppTask.DispatchEvent(&event);
            eventReceived = xQueueReceive(sAppEventQueue, &event, 0);
        }

        // Collect connectivity and configuration state from the CHIP stack.  Because the
        // CHIP event loop is being run in a separate task, the stack must be locked
        // while these values are queried.  However we use a non-blocking lock request
        // (TryLockChipStack()) to avoid blocking other UI activities when the CHIP
        // task is busy (e.g. with a long crypto operation).
        if (PlatformMgr().TryLockChipStack())
        {
#if CHIP_DEVICE_CONFIG_THREAD_ENABLE_CLI
            K32WUartProcess();
#endif
            sHaveBLEConnections  = (ConnectivityMgr().NumBLEConnections() != 0);
            PlatformMgr().UnlockChipStack();
        }

        // Update the status LED if factory reset has not been initiated.
        //
        // If system has "full connectivity", keep the LED On constantly.
        //
        // If thread and service provisioned, but not attached to the thread network yet OR no
        // connectivity to the service OR subscriptions are not fully established
        // THEN blink the LED Off for a short period of time.
        //
        // If the system has ble connection(s) uptill the stage above, THEN blink the LEDs at an even
        // rate of 100ms.
        //
        // Otherwise, blink the LED ON for a very short time.

#if !cPWR_UsePowerDownMode
        if (sAppTask.mFunction != kFunction_FactoryReset)
        {
            if (sIsThreadProvisioned)
            {
                sStatusLED.Blink(950, 50);
            }
            else if (sHaveBLEConnections)
            {
                sStatusLED.Blink(100, 100);
            }
            else
            {
                sStatusLED.Blink(50, 950);
            }
        }

        sStatusLED.Animate();
        sContactSensorLED.Animate();
#endif
    }
}

void AppTask::ButtonEventHandler(uint8_t pin_no, uint8_t button_action)
{
    if ((pin_no != RESET_BUTTON) && (pin_no != CONTACT_SENSOR_BUTTON) && (pin_no != OTA_BUTTON) && (pin_no != BLE_BUTTON))
    {
        return;
    }

    AppEvent button_event;
    button_event.Type               = AppEvent::kEventType_Button;
    button_event.ButtonEvent.PinNo  = pin_no;
    button_event.ButtonEvent.Action = button_action;

    if (pin_no == RESET_BUTTON)
    {
        button_event.Handler = ResetActionEventHandler;
    }
    else if (pin_no == CONTACT_SENSOR_BUTTON)
    {
        button_event.Handler = ContactActionEventHandler;
    }
    else if (pin_no == OTA_BUTTON)
    {
        button_event.Handler = OTAHandler;
    }
    else if (pin_no == BLE_BUTTON)
    {
        button_event.Handler = BleHandler;
#if !(defined OM15082)
        if (button_action == RESET_BUTTON_PUSH)
        {
            button_event.Handler = ResetActionEventHandler;
        }
#endif
    }

    sAppTask.PostEvent(&button_event);
}

void AppTask::KBD_Callback(uint8_t events)
{
    eventMask = eventMask | (uint32_t)(1 << events);

    HandleKeyboard();
}

void AppTask::HandleKeyboard(void)
{
    uint8_t keyEvent = 0xFF;
    uint8_t pos      = 0;

    while (eventMask)
    {
        for (pos = 0; pos < (8 * sizeof(eventMask)); pos++)
        {
            if (eventMask & (1 << pos))
            {
                keyEvent  = pos;
                eventMask = eventMask & ~(1 << pos);
                break;
            }
        }

        switch (keyEvent)
        {
        case gKBD_EventPB1_c:
            K32W_LOG("pb1 short press");

#if (defined OM15082)
            ButtonEventHandler(RESET_BUTTON, RESET_BUTTON_PUSH);
            break;
#else
            ButtonEventHandler(BLE_BUTTON, BLE_BUTTON_PUSH);
            break;
#endif
        case gKBD_EventPB2_c:
            ButtonEventHandler(CONTACT_SENSOR_BUTTON, CONTACT_SENSOR_BUTTON_PUSH);
            break;
        case gKBD_EventPB3_c:
            ButtonEventHandler(OTA_BUTTON, OTA_BUTTON_PUSH);
            break;
        case gKBD_EventPB4_c:
            ButtonEventHandler(BLE_BUTTON, BLE_BUTTON_PUSH);
            break;
#if !(defined OM15082)
        case gKBD_EventLongPB1_c:
            K32W_LOG("pb1 long press");
            ButtonEventHandler(BLE_BUTTON, RESET_BUTTON_PUSH);
            break;
#endif
        default:
            break;
        }
    }
}

void AppTask::TimerEventHandler(TimerHandle_t xTimer)
{
    AppEvent event;
    event.Type               = AppEvent::kEventType_Timer;
    event.TimerEvent.Context = (void *) xTimer;
    event.Handler            = FunctionTimerEventHandler;
    sAppTask.PostEvent(&event);
}

void AppTask::FunctionTimerEventHandler(void * aGenericEvent)
{
    AppEvent * aEvent = (AppEvent *) aGenericEvent;

    if (aEvent->Type != AppEvent::kEventType_Timer)
        return;

    K32W_LOG("Device will factory reset...");

    // Actually trigger Factory Reset
    chip::Server::GetInstance().ScheduleFactoryReset();
}

void AppTask::ResetActionEventHandler(void * aGenericEvent)
{
    AppEvent * aEvent = (AppEvent *) aGenericEvent;

    if (aEvent->ButtonEvent.PinNo != RESET_BUTTON && aEvent->ButtonEvent.PinNo != BLE_BUTTON)
        return;

    if (sAppTask.mResetTimerActive)
    {
        sAppTask.CancelTimer();
        sAppTask.mFunction = kFunction_NoneSelected;

#if !cPWR_UsePowerDownMode
        /* restore initial state for the LED indicating Lock state */
        if (ContactSensorMgr().IsUnlocked())
        {
            sContactSensorLED.Set(false);
        }
        else
        {
            sContactSensorLED.Set(true);
        }
#endif

        K32W_LOG("Factory Reset was cancelled!");
    }
    else
    {
        uint32_t resetTimeout = FACTORY_RESET_TRIGGER_TIMEOUT;

        if (sAppTask.mFunction != kFunction_NoneSelected)
        {
            K32W_LOG("Another function is scheduled. Could not initiate Factory Reset!");
            return;
        }

        K32W_LOG("Factory Reset Triggered. Push the RESET button within %lu ms to cancel!", resetTimeout);
        sAppTask.mFunction = kFunction_FactoryReset;

        /* LEDs will start blinking to signal that a Factory Reset was scheduled */
#if !cPWR_UsePowerDownMode
        sStatusLED.Set(false);
        sContactSensorLED.Set(false);

        sStatusLED.Blink(500);
        sContactSensorLED.Blink(500);
#endif

        sAppTask.StartTimer(FACTORY_RESET_TRIGGER_TIMEOUT);
    }
}

void AppTask::ContactActionEventHandler(void * aGenericEvent)
{
    AppEvent * aEvent = (AppEvent *) aGenericEvent;
    ContactSensorManager::Action_t action;
    CHIP_ERROR err = CHIP_NO_ERROR;
    int32_t actor  = 0;
    bool initiated = false;

    if (sAppTask.mFunction != kFunction_NoneSelected)
    {
        K32W_LOG("Another function is scheduled. Could not initiate Lock/Unlock!");
        return;
    }

    if (aEvent->Type == AppEvent::kEventType_Contact)
    {
        action = static_cast<ContactSensorManager::Action_t>(aEvent->ContactEvent.Action);
        actor  = aEvent->ContactEvent.Actor;
    }
    else if (aEvent->Type == AppEvent::kEventType_Button)
    {
        if (ContactSensorMgr().IsUnlocked())
        {
            action = ContactSensorManager::LOCK_ACTION;
        }
        else
        {
            action = ContactSensorManager::UNLOCK_ACTION;
        }
    }
    else
    {
        err    = APP_ERROR_UNHANDLED_EVENT;
        action = ContactSensorManager::INVALID_ACTION;
    }

    if (err == CHIP_NO_ERROR)
    {
        initiated = ContactSensorMgr().InitiateAction(actor, action);

        if (!initiated)
        {
            K32W_LOG("Action is already in progress or active.");
        }
    }
}

void AppTask::OTAHandler(void * aGenericEvent)
{
    AppEvent * aEvent = (AppEvent *) aGenericEvent;
    if (aEvent->ButtonEvent.PinNo != OTA_BUTTON)
        return;

#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
    if (sAppTask.mFunction != kFunction_NoneSelected)
    {
        K32W_LOG("Another function is scheduled. Could not initiate OTA!");
        return;
    }

    PlatformMgr().ScheduleWork(StartOTAQuery, 0);
#endif
}

#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
void AppTask::StartOTAQuery(intptr_t arg)
{
    static_cast<DefaultOTARequestor *>(GetRequestorInstance())->TriggerImmediateQuery();
}

void AppTask::PostOTAResume()
{
    AppEvent event;
    event.Type    = AppEvent::kEventType_OTAResume;
    event.Handler = OTAResumeEventHandler;
    sAppTask.PostEvent(&event);
}
#endif

void AppTask::BleHandler(void * aGenericEvent)
{
    AppEvent * aEvent = (AppEvent *) aGenericEvent;

    if (aEvent->ButtonEvent.PinNo != BLE_BUTTON)
        return;

    if (sAppTask.mFunction != kFunction_NoneSelected)
    {
        K32W_LOG("Another function is scheduled. Could not toggle BLE state!");
        return;
    }

    if (ConnectivityMgr().IsBLEAdvertisingEnabled())
    {
        ConnectivityMgr().SetBLEAdvertisingEnabled(false);
    #if !cPWR_UsePowerDownMode
        sStatusLED.Set(false);
    #endif
        K32W_LOG("Stopped BLE Advertising!");
    }
    else
    {
        ConnectivityMgr().SetBLEAdvertisingEnabled(true);

        if (chip::Server::GetInstance().GetCommissioningWindowManager().OpenBasicCommissioningWindow() == CHIP_NO_ERROR)
        {
        #if !cPWR_UsePowerDownMode
            sStatusLED.Set(true);
        #endif
            K32W_LOG("Started BLE Advertising!");
        }
        else
        {
            K32W_LOG("OpenBasicCommissioningWindow() failed");
        }
    }
}

void AppTask::ThreadProvisioningHandler(const ChipDeviceEvent * event, intptr_t)
{
    if (event->Type == DeviceEventType::kServiceProvisioningChange && event->ServiceProvisioningChange.IsServiceProvisioned)
    {
        if (event->ServiceProvisioningChange.IsServiceProvisioned)
        {
            sIsThreadProvisioned = TRUE;
        }
        else
        {
            sIsThreadProvisioned = FALSE;
        }
    }

#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
    if (event->Type == DeviceEventType::kOtaStateChanged && event->OtaStateChanged.newState == kOtaSpaceAvailable)
    {
        sAppTask.PostOTAResume();
    }
#endif

#if CONFIG_CHIP_NFC_COMMISSIONING
    if (event->Type == DeviceEventType::kCHIPoBLEAdvertisingChange && event->CHIPoBLEAdvertisingChange.Result == kActivity_Stopped)
    {
        if (!NFCMgr().IsTagEmulationStarted())
        {
            K32W_LOG("NFC Tag emulation is already stopped!");
        }
        else
        {
            NFCMgr().StopTagEmulation();
            K32W_LOG("Stopped NFC Tag Emulation!");
        }
    }
    else if (event->Type == DeviceEventType::kCHIPoBLEAdvertisingChange &&
             event->CHIPoBLEAdvertisingChange.Result == kActivity_Started)
    {
        if (NFCMgr().IsTagEmulationStarted())
        {
            K32W_LOG("NFC Tag emulation is already started!");
        }
        else
        {
            ShareQRCodeOverNFC(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
            K32W_LOG("Started NFC Tag Emulation!");
        }
    }
#endif
}

void AppTask::CancelTimer()
{
    if (xTimerStop(sFunctionTimer, 0) == pdFAIL)
    {
        K32W_LOG("app timer stop() failed");
    }

    mResetTimerActive = false;
}

void AppTask::StartTimer(uint32_t aTimeoutInMs)
{
    if (xTimerIsTimerActive(sFunctionTimer))
    {
        K32W_LOG("app timer already started!");
        CancelTimer();
    }

    // timer is not active, change its period to required value (== restart).
    // FreeRTOS- Block for a maximum of 100 ticks if the change period command
    // cannot immediately be sent to the timer command queue.
    if (xTimerChangePeriod(sFunctionTimer, aTimeoutInMs / portTICK_PERIOD_MS, 100) != pdPASS)
    {
        K32W_LOG("app timer start() failed");
    }

    mResetTimerActive = true;
}

void AppTask::ActionInitiated(ContactSensorManager::Action_t aAction, int32_t aActor)
{
    // If the action has been initiated by the lock, update the bolt lock trait
    // and start flashing the LEDs rapidly to indicate action initiation.
    if (aAction == ContactSensorManager::LOCK_ACTION)
    {
        K32W_LOG("Lock Action has been initiated")
    }
    else if (aAction == ContactSensorManager::UNLOCK_ACTION)
    {
        K32W_LOG("Unlock Action has been initiated")
    }

    if (aActor == AppEvent::kEventType_Button)
    {
        sAppTask.mSyncClusterToButtonAction = true;
    }

    sAppTask.mFunction = kFunctionLockUnlock;

#if !cPWR_UsePowerDownMode
    sContactSensorLED.Blink(50, 50);
#endif
}

void AppTask::ActionCompleted(ContactSensorManager::Action_t aAction)
{
    // if the action has been completed by the lock, update the bolt lock trait.
    // Turn on the lock LED if in a LOCKED state OR
    // Turn off the lock LED if in an UNLOCKED state.
    if (aAction == ContactSensorManager::LOCK_ACTION)
    {
        K32W_LOG("Lock Action has been completed")
#if !cPWR_UsePowerDownMode
        sContactSensorLED.Set(true);
#endif
    }
    else if (aAction == ContactSensorManager::UNLOCK_ACTION)
    {
        K32W_LOG("Unlock Action has been completed")
#if !cPWR_UsePowerDownMode
        sContactSensorLED.Set(false);
#endif
    }

    if (sAppTask.mSyncClusterToButtonAction)
    {
        sAppTask.UpdateClusterState();
        sAppTask.mSyncClusterToButtonAction = false;
    }

    sAppTask.mFunction = kFunction_NoneSelected;
}

void AppTask::PostContactActionRequest(int32_t aActor, ContactSensorManager::Action_t aAction)
{
    AppEvent event;
    event.Type             = AppEvent::kEventType_Contact;
    event.ContactEvent.Actor  = aActor;
    event.ContactEvent.Action = aAction;
    event.Handler          = ContactActionEventHandler;
    PostEvent(&event);
}

void AppTask::OTAResumeEventHandler(void * aGenericEvent)
{
    AppEvent * aEvent = (AppEvent *) aGenericEvent;
    if (aEvent->Type == AppEvent::kEventType_OTAResume)
    {
#if CHIP_DEVICE_CONFIG_ENABLE_OTA_REQUESTOR
        if (gDownloader.GetState() == OTADownloader::State::kInProgress)
        {
            gImageProcessor.TriggerNewRequestForData();
        }
#endif
    }
}

void AppTask::PostEvent(const AppEvent * aEvent)
{
    portBASE_TYPE taskToWake = pdFALSE;
    if (sAppEventQueue != NULL)
    {
        if (__get_IPSR())
        {
            if (!xQueueSendToFrontFromISR(sAppEventQueue, aEvent, &taskToWake))
            {
                K32W_LOG("Failed to post event to app task event queue");
            }
            if (taskToWake)
            {
                portYIELD_FROM_ISR(taskToWake);
            }
        }
        else if (!xQueueSend(sAppEventQueue, aEvent, 0))
        {
            K32W_LOG("Failed to post event to app task event queue");
        }
    }
}

void AppTask::DispatchEvent(AppEvent * aEvent)
{
#if defined(cPWR_UsePowerDownMode) && (cPWR_UsePowerDownMode)
    /* specific processing for events sent from App_PostCallbackMessage (see main.cpp) */
    if (aEvent->Type == AppEvent::kEventType_Lp)
    {
        aEvent->Handler(aEvent->param);
    }
#endif

    if (aEvent->Handler)
    {
        aEvent->Handler(aEvent);
    }
    else
    {
        K32W_LOG("Event received with no handler. Dropping event.");
    }
}

void AppTask::UpdateClusterState(void)
{
    PlatformMgr().ScheduleWork(UpdateClusterStateInternal, 0);
}

void AppTask::UpdateClusterStateInternal(intptr_t arg)
{
    uint8_t newValue = !ContactSensorMgr().IsUnlocked();

    // write the new on/off value
    EmberAfStatus status = emberAfWriteAttribute(1, ZCL_BOOLEAN_STATE_CLUSTER_ID, ZCL_STATE_VALUE_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                                 (uint8_t *) &newValue, ZCL_BOOLEAN_ATTRIBUTE_TYPE);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        ChipLogError(NotSpecified, "ERR: updating boolean status value %x", status);
    }
}

void AppTask::UpdateDeviceState(void)
{
    PlatformMgr().ScheduleWork(UpdateDeviceStateInternal, 0);
}

void AppTask::UpdateDeviceStateInternal(intptr_t arg)
{
    bool stateValueAttrValue  = 0;

    /* get onoff attribute value */
    (void)emberAfReadAttribute(1, ZCL_BOOLEAN_STATE_CLUSTER_ID, ZCL_STATE_VALUE_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
            (uint8_t *) &stateValueAttrValue, 1, NULL);
#if !cPWR_UsePowerDownMode
    /* set the device state */
    sContactSensorLED.Set(stateValueAttrValue);
#endif
}
