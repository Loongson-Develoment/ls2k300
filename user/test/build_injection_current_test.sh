#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
TARGET="injection_current_test"
REMOTE_HOST="${REMOTE_HOST:-root@192.168.214.100}"
REMOTE_TARGET="${REMOTE_TARGET:-~/injection_current_test}"
REMOTE_TEMP="${REMOTE_TARGET}.tmp.$$"

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target "${TARGET}" -j"${JOBS:-8}"
cp "${BUILD_DIR}/${TARGET}" "${SCRIPT_DIR}/${TARGET}"
scp "${SCRIPT_DIR}/${TARGET}" "${REMOTE_HOST}:${REMOTE_TEMP}"
ssh "${REMOTE_HOST}" "mv ${REMOTE_TEMP} ${REMOTE_TARGET}"

echo "built: ${SCRIPT_DIR}/${TARGET}"
