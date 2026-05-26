# INFO - AZURE_SEQUENCER

## Target

- Board: Raspberry Pi Pico 2 / RP2350
- Main firmware target: `hrdw_test`
- Build system: CMake + Pico SDK

## Default Calibration Codes

Per channel default points used by calibration:

- `-3V` -> `0000`
- `0V` -> `1220`
- `+3V` -> `2453`
- `+6V` -> `3680`

Calibration is CRC-protected and stored in flash.

## Controls Summary

- `ENC1` (left): menu/parameter selection
- `ENC2` (right): parameter value edit
- `ENC_R_SW`: save in calibration and app settings contexts
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
