#!/usr/bin/env python3
#
#    Copyright (c) 2022 Project CHIP Authors
#    All rights reserved.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
#

import os
import sys
import argparse
import subprocess
import logging
import hashlib

from default import InputArgument
from custom import *

def set_logger():
    stdout_handler = logging.StreamHandler(stream=sys.stdout)
    logging.basicConfig(
        level=logging.DEBUG, 
        format='[%(levelname)s] %(message)s',
        handlers=[stdout_handler]
    )

class Spake2p:

    def __init__(self):
        pass
    
    def generate(self, args):
        params = self._generate_params(args)
        args.spake2p_verifier = Verifier(params["Verifier"])
        args.salt = Salt(params["Salt"])
        args.it = IterationCount(params["Iteration Count"])

    def _generate_params(self, args):
        cmd = [
            args.spake2p_path, "gen-verifier",
            "--iteration-count", str(args.it.val),
            "--salt", args.salt.encode(),
            "--pin-code", args.passcode,
            "--out", "-"
        ]

        out = subprocess.run(cmd, check=True, stdout=subprocess.PIPE).stdout
        out = out.decode("utf-8").splitlines()
        return dict(zip(out[0].split(','), out[1].split(',')))

class KlvGenerator:

    def __init__(self, args):
        self.args = args
        self._validate_args()
        self.spake2p = Spake2p()
        if self.args.spake2p_verifier is None:
            self.spake2p.generate(self.args)
        self.args.dac_key.generate_private_key(self.args.dac_key_password)
    
    def _validate_args(self):
        if self.args.dac_key_password is None:
            logging.warning(
                "DAC Key password not provided. It means DAC Key is not protected."
            )

    def generate(self):
        '''Return a list of (K, L, V) tuples.

        args is essentially a dict, so the entries are not ordered.
        Sort the objects to ensure the same order of KLV data every
        time (sorted by key), thus ensuring that SHA256 can be used
        correctly to compare two output binaries.

        The new list will contain only InputArgument objects, which
        generate a (K, L, V) tuple through output() method.
        '''
        data = list()

        data = [obj for key, obj in vars(self.args).items() if isinstance(obj, InputArgument)]
        data = [arg.output() for arg in sorted(data, key=lambda x: x.key())]

        return data

    def to_bin(self, klv, out, aes128_key):
        fullContent = bytearray()
        with open(out, "wb") as file:
            for entry in klv:
                fullContent += entry[0].to_bytes(1, "little")
                fullContent += entry[1].to_bytes(2, "little")
                fullContent += entry[2]
            size = len(fullContent)
            if (aes128_key is None):
                file.write(fullContent)
                logging.info("Size of generated binary is: {} bytes".format(size))
            else:
                #In case a aes128_key is given the data will be encrypted
                #Always add a padding to be 16 bytes aligned
                padding_len = size % 16
                padding_len = 16 - padding_len
                padding_bytes = bytearray(padding_len)
                logging.info("(Before padding) Size of generated binary is: {} bytes".format(size))
                fullContent += padding_bytes
                size = len(fullContent)
                logging.info("(After padding) Size of generated binary is: {} bytes".format(size))
                from Crypto.Cipher import AES
                cipher = AES.new(bytes.fromhex(aes128_key), AES.MODE_ECB)
                fullContentCipher = cipher.encrypt(fullContent)
                file.write(fullContentCipher)

        out_hash = hashlib.sha256(fullContent).hexdigest()
        logging.info("SHA256 of generated binary: {}".format(out_hash))

def main():
    set_logger()
    parser = argparse.ArgumentParser(description="NXP Factory Data Generator")
    optional = parser
    required = parser.add_argument_group("required arguments")

    required.add_argument("-i", "--it", required=True, type=IterationCount,
                        help="[int | hex] Spake2 Iteration Counter")
    required.add_argument("-s", "--salt", required=True, type=Salt,
                        help="[base64 str] Spake2 Salt")
    required.add_argument("-p", "--passcode", required=True, type=str,
                        help="[int | hex] PASE session passcode")
    required.add_argument("-d", "--discriminator", required=True, type=Discriminator,
                        help="[int | hex] BLE Pairing discriminator")
    required.add_argument("--vid", required=True, type=VendorId,
                        help="[int | hex] Vendor Identifier (VID)")
    required.add_argument("--pid", required=True, type=ProductId,
                        help="[int | hex] Product Identifier (PID)")
    required.add_argument("--vendor_name", required=True, type=VendorName,
                        help="[str] Vendor Name")
    required.add_argument("--product_name", required=True, type=ProductName,
                        help="[str] Product Name")
    required.add_argument("--serial_num", required=True, type=SerialNum,
                        help="[str] Serial Number")
    required.add_argument("--date", required=True, type=ManufacturingDate,
                        help="[str] Manufacturing Date (YYYY-MM-DD)")
    required.add_argument("--hw_version", required=True, type=HardwareVersion,
                        help="[int | hex] Hardware version as number")
    required.add_argument("--hw_version_str", required=True, type=HardwareVersionStr,
                        help="[str] Hardware version as string")
    required.add_argument("--cert_declaration", required=True, type=CertDeclaration,
                        help="[path] Path to Certification Declaration in DER format")
    required.add_argument("--dac_cert", required=True, type=DacCert,
                        help="[path] Path to DAC certificate in DER format")
    required.add_argument("--dac_key", required=True, type=DacPKey,
                        help="[path] Path to DAC key in DER format")
    required.add_argument("--pai_cert", required=True, type=PaiCert,
                        help="[path] Path to PAI certificate in DER format")
    required.add_argument("--spake2p_path", required=True, type=str,
                        help="[path] Path to spake2p tool")
    required.add_argument("--out", required=True, type=str,
                        help="[path] Path to output binary")

    optional.add_argument("--dac_key_password", type=str,
                        help="[path] Password to decode DAC Key if available")
    optional.add_argument("--spake2p_verifier", type=Verifier,
                        help="[base64 str] Already generated spake2p verifier")
    optional.add_argument("--aes128_key",
                        help="[hex] AES 128 bits key used to encrypt the whole dataset")

    args = parser.parse_args()

    klv = KlvGenerator(args)
    data = klv.generate()
    klv.to_bin(data, args.out, args.aes128_key)

if __name__ == "__main__":
    main()
