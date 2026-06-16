# Repository Guidelines

## Project Structure & Module Organization

This repository contains control and vision code for the `rescue_X` car.

- `car_control/`: ESP32/Arduino vehicle control firmware. The main sketch is `car_control.ino`.
- `MaixCam/`: MaixCam Python code for YOLO-based ball and safe-area detection plus control logic. Helpers such as `uartCommand.py` and `control_servo.py` run on the device.
- `K230/`: CanMV K230 vision code. The current main line is `yolo_sender.py` (YOLO11s detection + UART output to ESP32) and `yolo_test.py` (standalone preview/test). `color_vision.py` remains as a traditional LAB color-vision prototype with `color_config.py`. `rescue_protocol.py` is the shared UART frame encoder.
- `model/`: Model artifacts (`.mud`, `.cvimodel`, `.kmodel`) and `data.yaml`. Keep generated model files here.

There is currently no dedicated `tests/` directory.

## Build, Test, and Development Commands

- `python -m py_compile K230/yolo_sender.py K230/yolo_test.py K230/rescue_protocol.py K230/color_vision.py K230/color_config.py`: quick syntax check for K230 Python.
- `python -m py_compile MaixCam/*.py`: quick syntax check for MaixCam scripts; hardware-specific imports may still require device validation.
- Open `car_control/car_control.ino` in Arduino IDE or PlatformIO and build for ESP32 to validate firmware changes.
- Run `K230/yolo_test.py` in CanMV IDE K230 to verify YOLO detection before connecting ESP32.
- Run `K230/yolo_sender.py` for real-car deployment after ESP32 wiring is complete.
- Run `K230/color_vision.py` in CanMV IDE K230 with `color_config.py` in the same directory.

Deploy MaixCam helpers according to `README.md`: place `uartCommand.py` and `control_servo.py` under the runtime path expected by the device.

## Coding Style & Naming Conventions

Use 4-space indentation for Python and keep constants in `UPPER_SNAKE_CASE`, especially thresholds. Prefer descriptive snake_case for functions and variables. Keep pins, thresholds, and model paths grouped near the top of a file or in config modules.

For Arduino/C++, follow the existing sketch style: `#define` constants for pins and protocol values, concise helpers, and comments for hardware-specific behavior. Avoid unrelated formatting churn.

## Testing Guidelines

Automated tests are not configured. For Python changes, run `py_compile` before deployment. For vision changes, validate on the target device with representative lighting, distance, shadows, and moving targets. For firmware changes, build for ESP32 and test with wheels raised before field testing.

Record any changed thresholds, model filenames, or wiring assumptions in the relevant README or source comments.

## Commit & Pull Request Guidelines

Recent commit messages are short, imperative summaries such as `add source files` and `compatible with K230`. Keep future commits similarly concise, but make them specific: `tune K230 purple threshold`, `fix ESP32 UART frame length`.

Pull requests should include a short description, affected hardware (`ESP32`, `MaixCam`, `K230`), validation performed, and any required deployment path changes. Add screenshots or serial output snippets when changing vision detection or UART protocols.
