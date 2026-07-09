#!/bin/bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

make -C "$SCRIPT_DIR/build"

scp *.ko root@192.168.148.100:/home/root/driver/