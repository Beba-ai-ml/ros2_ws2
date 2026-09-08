# VESC configuration tool

`vesc_config_upload.py` uploads a motor (`MCConfiguration`) and/or app
(`APPConfiguration`) XML — exported from VESC Tool — to the VESC over the serial
link, without needing VESC Tool itself. Handy on the Jetson, where VESC Tool is
not installed.

Tested against **VESC firmware 6.02 on HW60** (the board in this car).

## Contents

| Path | What it is |
| --- | --- |
| `vesc_config_upload.py` | The uploader. Pure Python + `pyserial`. |
| `configs/motor_config.xml` | This car's motor configuration (MCConfiguration). |
| `configs/app_config.xml` | This car's app configuration (APPConfiguration, PPM/UART setup). |
| `params/6.02/parameters_mcconf.xml` | FW 6.02 motor parameter definitions (field order + types). |
| `params/6.02/parameters_appconf.xml` | FW 6.02 app parameter definitions. |
| `params/6.02/info.xml` | FW 6.02 parameter descriptions (reference only). |

The `params/6.02/` XMLs come from `vesc_tool/res/config/6.02`. They define the
exact serialization order and the config *signature* the firmware checks, so they
must match the firmware on the board. For a different firmware version, drop its
`parameters_*.xml` next to a new `params/<version>/` directory and point
`--mcconf-params` / `--appconf-params` at it.

## Prerequisites

- `pip3 install --user pyserial` (already in the repo `requirements.txt`)
- The VESC plugged in over USB. The udev rule in `system/udev/99-f1tenth.rules`
  gives it the stable symlink `/dev/vesc` at mode 0666; check with
  `ls -l /dev/vesc`.
- Nothing else talking to the VESC — stop bringup / `vesc_driver_node` first.
- **Wheels off the ground.** Writing a motor config can spin the motor.

## Usage

Run from the workspace root (`~/ros2_ws`):

```bash
python3 tools/vesc/vesc_config_upload.py \
    --motor tools/vesc/configs/motor_config.xml \
    --app   tools/vesc/configs/app_config.xml \
    --verify
```

Default port is `/dev/vesc` at `115200` baud; override with `--port` / `--baud`.

## Flags

| Flag | Meaning |
| --- | --- |
| `--motor FILE` | Motor config XML to upload. |
| `--app FILE` | App config XML to upload. |
| `--port PORT` | Serial port (default `/dev/vesc`). |
| `--baud N` | Baud rate (default `115200`). |
| `--dry-run` | Parse and serialize the config, print what *would* be sent, and exit without opening the serial port. Nothing touches the VESC. Use this first. |
| `--verify` | After writing, read the config back from the VESC and compare it field by field. Reports any mismatch. |
| `--check-sig` | Only compute the config signature(s) and print them. Use it to confirm the `params/` definitions match the firmware on the board before writing anything. |
| `--mcconf-params FILE` | Override the motor parameter-definitions XML (default: `params/6.02/parameters_mcconf.xml`). |
| `--appconf-params FILE` | Override the app parameter-definitions XML (default: `params/6.02/parameters_appconf.xml`). |

At least one of `--motor`, `--app` or `--check-sig` is required.

## Suggested order

```bash
# 1. Is the board's firmware what we think it is?
python3 tools/vesc/vesc_config_upload.py --check-sig

# 2. What would we send?
python3 tools/vesc/vesc_config_upload.py --motor tools/vesc/configs/motor_config.xml --dry-run

# 3. Send it and read it back.
python3 tools/vesc/vesc_config_upload.py \
    --motor tools/vesc/configs/motor_config.xml \
    --app   tools/vesc/configs/app_config.xml --verify
```

If the signature check fails, the parameter definitions do not match the
firmware — do **not** write the config, or you will end up with garbage values in
the wrong fields. Get the matching `parameters_*.xml` for your firmware first.
