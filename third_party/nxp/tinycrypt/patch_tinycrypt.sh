#!/bin/bash

if [[ ! -d $PW_PROJECT_ROOT ]]; then
    echo "PW_PROJECT_ROOT is not set, run activate.sh script"
    exit 1
fi

SOURCE=${BASH_SOURCE[0]}
SOURCE_DIR=$(cd "$(dirname "$SOURCE")" >/dev/null 2>&1 && pwd)

# Mbedtls

patch -N --binary -d "$PW_PROJECT_ROOT"/third_party/mbedtls/repo -p1 <"$SOURCE_DIR/mbedtls_tinycrypt.patch"

# Openthread

patch -N --binary -d "$PW_PROJECT_ROOT"/third_party/openthread/repo -p1 <"$SOURCE_DIR/openthread_ecdsa_cpp.patch"

# Tinycrypt changes

if [ ! -d "$PW_PROJECT_ROOT"/third_party/mbedtls/repo/include/tinycrypt ]; then
	cp -r "$SOURCE_DIR"/new_files/tinycrypt_h "$PW_PROJECT_ROOT"/third_party/mbedtls/repo/include/tinycrypt
else
	echo "$PW_PROJECT_ROOT/third_party/mbedtls/repo/include/tinycrypt directory already exists."
fi

if [ ! -d "$PW_PROJECT_ROOT"/third_party/mbedtls/repo/tinycrypt ]; then
	cp -r "$SOURCE_DIR"/new_files/tinycrypt_c "$PW_PROJECT_ROOT"/third_party/mbedtls/repo/tinycrypt
else
	echo "$PW_PROJECT_ROOT/third_party/mbedtls/repo/tinycrypt directory already exists."
fi

echo "Tinycrypt patching done."
exit 0
