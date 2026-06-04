# INFO - AZURE_SEQUENCER

## Target

- Board: Raspberry Pi Pico 2 / RP2350
- Main firmware target: `hrdw_test`
- Build system: CMake + Pico SDK

## Default Calibration Codes

Per channel calibration stores separate point tables for both ranges:

- `-3..+7V`: `-3V`, `0V`, `+3V`, `+7V`
- `0..+10V`: `0V`, `+3V`, `+6V`, `+10V`

Fresh defaults are ideal linear 10 V span codes:

- `0000`
- `1229`
- `2457`
- `4095`

Calibration is CRC-protected and stored in flash.

## Controls Summary

- `ENC1` (left): menu/parameter selection
- `ENC2` (right): parameter value edit
- `SW1` / `SW2`: app screen navigation; the last two preset-enabled app screens are `LOAD` and `SAVE`
- `ENC_R_SW`: preset popup (`LOAD` / `SAVE`) on normal preset-enabled app screens
- `ENC_L_SW`: back to app menu

## GRIDS Screen Layout (current)

- Header + compact parameter menu:
  - `CLK / BPM`
  - `MAPX / MAPY`
  - `CHA`
- Under the menu:
  - 4 channels
  - each channel displayed as 32 steps (`2 x 16` cells)
  - active step marker shown as a bottom bar in the current cell
  - active trigger cells shown with filled center square

## Notes

- Output trigger pulse width is software-timed and serviced before and after UI draw to reduce timing stretch.
- OLED rendering uses incremental cell updates in grid screens to reduce flicker.
