#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

OPENOCD_BIN="${HOME}/Library/Arduino15/packages/rp2040/tools/pqt-openocd/4.1.0-1aec55e/bin/openocd"
OPENOCD_SCRIPTS="${HOME}/Library/Arduino15/packages/rp2040/tools/pqt-openocd/4.1.0-1aec55e/share/openocd/scripts"
BUILD_DIR="${ROOT_DIR}/build"
ELF_PATH="${BUILD_DIR}/hrdw_test.elf"

if [[ ! -x "${OPENOCD_BIN}" ]]; then
  echo "ERROR: Nie znaleziono openocd: ${OPENOCD_BIN}" >&2
  echo "Uruchom Arduino RP2040 toolchain albo popraw sciezke w skrypcie." >&2
  exit 1
fi

cd "${ROOT_DIR}"
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" -j4

"${OPENOCD_BIN}" \
  -s "${OPENOCD_SCRIPTS}" \
  -f interface/cmsis-dap.cfg \
  -f target/rp2350.cfg \
  -c "transport select swd" \
  -c "cortex_m reset_config sysresetreq" \
  -c "adapter speed 500" \
  -c "program ${ELF_PATH} verify reset exit"
