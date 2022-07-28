/*
 *
 *    Copyright (c) 2022 Project CHIP Authors
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

/**
 *    @file
 *          Provides a glue layer between Matter and NXP-SDK Low Power
 */

#if defined(cPWR_UsePowerDownMode) && (cPWR_UsePowerDownMode)

#include <platform/CHIPDeviceLayer.h>
#include <platform/ThreadStackManager.h>

#include "PWR_Configuration.h"
#include "PWR_Interface.h"
#include "app_dual_mode_low_power.h"
#include "app_dual_mode_switch.h"
#include "OtaSupport.h"
#include "Keyboard.h"
#include "MacSched.h"
#include "RNG_Interface.h"
#include "k32w0-chip-mbedtls-config.h"
#include <AppTask.h>

using namespace ::chip;
using namespace ::chip::Inet;
using namespace ::chip::DeviceLayer;

extern "C" void InitLowPower();

extern "C" void vMMAC_IntHandlerBbc();
extern "C" void vMMAC_IntHandlerPhy();
extern "C" void BOARD_SetClockForPowerMode(void);
extern "C" void stopM2();
extern "C" void sched_enable();

static void dm_switch_wakeupCallBack(void);
static void dm_switch_preSleepCallBack(void);
static void ThreadExitSleep();
static void BOARD_SetClockForWakeup(void);

typedef struct
{
    bool_t bleAppRunning;
    bool_t bleAppStopInprogress;
    bool_t threadInitialized;
    uint32_t threadWarmBootInitTime;
} sDualModeAppStates;

typedef void * appCallbackParam_t;
typedef void (*appCallbackHandler_t)(appCallbackParam_t param);

static sDualModeAppStates dualModeStates;
constexpr uint16_t kThreadWarmDefaultValue  = 4000;

extern "C" void otTaskletsSignalPending(otInstance * p_instance);

#define ENABLE_LOW_POWER_LOGS 0

void InitLowPower()
{
    PWR_Init();

    /* Internal - MATTER-303: keep in retention the entire RAM1 for the moment */
    PWR_vAddRamRetention((uint32_t)0x4020000, 0x10000);

    PWR_RegisterLowPowerExitCallback(dm_switch_wakeupCallBack);
    PWR_RegisterLowPowerEnterCallback(dm_switch_preSleepCallBack);

    dualModeStates.threadWarmBootInitTime = kThreadWarmDefaultValue;

    dm_lp_init();
}

extern "C" void setThreadInitialized(bool isInitialized)
{
    dualModeStates.threadInitialized = isInitialized;
}

extern "C" bool isThreadInitialized()
{
    return dualModeStates.threadInitialized;
}

uint32_t dm_switch_get15_4InitWakeUpTime(void)
{
    return dualModeStates.threadWarmBootInitTime;
}

extern "C" bleResult_t App_PostCallbackMessage(appCallbackHandler_t handler, appCallbackParam_t param)
{
    AppEvent event;
    event.Type    = AppEvent::kEventType_Lp;
    event.Handler = handler;
    event.param   = param;

#if ENABLE_LOW_POWER_LOGS
    K32W_LOG("App_PostCallbackMessage %d", (uint32_t)param);
#endif

    GetAppTask().PostEvent(&event);

    return gBleSuccess_c;
}

static void dm_switch_wakeupCallBack(void)
{
    BOARD_SetClockForWakeup();
    SHA_ClkInit(SHA_INSTANCE);
    CLOCK_EnableClock(kCLOCK_Aes);

#if ENABLE_LOW_POWER_LOGS
    K32W_LOG("dm_switch_wakeupCallBack");
    K32W_LOG("Warm up time actual value: %d", dualModeStates.threadWarmBootInitTime);
#endif

    RNG_Init();
    SecLib_Init();
    OTA_InitExternalMemory();

    KBD_PrepareExitLowPower();

    PWR_WakeupReason_t wakeReason = PWR_GetWakeupReason();
    if (wakeReason.Bits.FromBLE_LLTimer == 1)
    {
#if ENABLE_LOW_POWER_LOGS
        K32W_LOG("woken up from LL");
#endif
    }
    else if (wakeReason.Bits.FromKeyBoard == 1)
    {
#if ENABLE_LOW_POWER_LOGS
        K32W_LOG("woken up from FromKeyBoard");
#endif
    }
    else if (wakeReason.Bits.FromTMR == 1)
    {
#if ENABLE_LOW_POWER_LOGS
        K32W_LOG("woken up from TMR");
#endif
    }
    dm_lp_wakeup();
}


static void dm_switch_preSleepCallBack(void)
{
#if ENABLE_LOW_POWER_LOGS
    K32W_LOG("dm_switch_preSleepCallBack");
#endif

    if (dualModeStates.threadInitialized)
    {
        /* stop the internal MAC Scheduler timer */
        stopM2();
        /* disable the MAC scheduler */
        sched_disable();
        otPlatRadioDisable(NULL);
        setThreadInitialized(FALSE);
    }
    /* Inform the low power dual mode module that we will sleep */
    dm_lp_preSleep();

    EEPROM_DeInit();

    /* disable SHA clock */
    SHA_ClkDeinit(SHA_INSTANCE);

    /* disable the AES clock */
    CLOCK_DisableClock(kCLOCK_Aes);

    /* configure pins for power down mode */
    BOARD_SetPinsForPowerMode();
    /* DeInitialize application support for drivers */
    BOARD_DeInitAdc();
    /* DeInit the necessary clocks */
    BOARD_SetClockForPowerMode();
}

extern "C" void vDynStopAll(void)
{
    vDynRequestState(E_DYN_SLAVE, E_DYN_STATE_OFF);
    vDynRequestState(E_DYN_MASTER, E_DYN_STATE_OFF);
}

void dm_switch_init15_4AfterWakeUp(void)
{
    uint32_t tick1 = 0;
    uint32_t tick2 = 0;

    if (dualModeStates.threadWarmBootInitTime == kThreadWarmDefaultValue)
    {
        /* Get a 32K tick */
        PWR_Start32kCounter();
        tick1 = PWR_Get32kTimestamp();
    }

    /* Radio must be re-enabled after waking up from sleep.
     * The module is completely disabled in power down mode */
    ThreadExitSleep();

    if (dualModeStates.threadWarmBootInitTime == kThreadWarmDefaultValue)
    {
        tick2                                 = PWR_Get32kTimestamp();
        dualModeStates.threadWarmBootInitTime = ((tick2 - tick1) * 15625u) >> 9;

        /* Add a margin of 1 ms */
        dualModeStates.threadWarmBootInitTime += 1000;

#if ENABLE_LOW_POWER_LOGS
        K32W_LOG("Calibration: %d", dualModeStates.threadWarmBootInitTime);
#endif
    }
}

static void ThreadExitSleep()
{
    if (!dualModeStates.threadInitialized && ConnectivityMgr().IsThreadEnabled())
    {
        /* Install uMac interrupt handlers */
        OSA_InstallIntHandler(ZIGBEE_MAC_IRQn, vMMAC_IntHandlerBbc);
        OSA_InstallIntHandler(ZIGBEE_MODEM_IRQn, vMMAC_IntHandlerPhy);

        /* Radio must be re-enabled after waking up from sleep.
         * The module is completely disabled in power down mode */
        otPlatRadioEnable(NULL);
        sched_enable();

        setThreadInitialized(TRUE);

        /* wake up the Thread stack and check if any processing needs to be done */
        otTaskletsSignalPending(NULL);
    }
}

extern "C" void App_NotifyWakeup(void)
{
    if ((!dualModeStates.threadInitialized))
    {

        /* Notify the dual mode low power mode */
        (void) App_PostCallbackMessage(dm_lp_processEvent, (void *) e15_4WakeUpEnded);
    }
}

extern "C" void App_AllowDeviceToSleep()
{
    PWR_PreventEnterLowPower(false);
}

extern "C" void App_DisallowDeviceToSleep()
{
    PWR_PreventEnterLowPower(true);
}

static void BOARD_SetClockForWakeup(void)
{
    /* Enables the clock for the I/O controller block. 0: Disable. 1: Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);
    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);
}

#endif
