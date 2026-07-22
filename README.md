# Autonomous Maze Runner RC Car

Embedded Systems term project code for an autonomous RC car that navigates a maze using LD19 LiDAR, a rear ultrasonic sensor, a LIS2DH12 accelerometer, PWM motor control, and UART/Bluetooth telemetry.

## Features

- Right-hand wall-following maze navigation
- LD19 LiDAR packet parsing over SERCOM2 UART
- Rear ultrasonic collision avoidance through EIC edge interrupts
- LIS2DH12 accelerometer reads through SERCOM3 I2C interrupts
- TCC0 PWM motor control for differential drive
- CSV telemetry output over SERCOM5 UART or HC-06 Bluetooth
- Python utilities for plotting speed, acceleration, and distance logs

## Repository Layout

```text
.
|-- firmware/
|   |-- main.c
|   |-- ld19.h
|   |-- uart_print.h
|   |-- asm_test.S
|   |-- GccApplication3.cproj
|   |-- GccApplication3.componentinfo.xml
|   `-- Device_Startup/
|       |-- startup_samd21.c
|       |-- system_samd21.c
|       |-- samd21g18a_flash.ld
|       `-- samd21g18a_sram.ld
|-- tools/
|   |-- plot_speed_csv_for_ppt.py
|   |-- live_speed_plot.py
|   `-- plot_saved_log.py
`-- reference/
    |-- project_base_bluetooth.c
    `-- project_base_uart.c
```

## Firmware

The main firmware target is `firmware/main.c`. It is written for the ATSAMD21G18A and was developed with Atmel Studio / Microchip Studio using ARM GCC.

The key output selector is:

```c
#define USE_BLUETOOTH_OUTPUT 0
```

Use `0` for PC UART debugging at 115200 bps, or `1` for HC-06 Bluetooth logging at 9600 bps.

## Telemetry Format

The firmware emits CSV rows in this format:

```text
CSV,time_ms,speed_cms,turn_count,front_cm,right_cm,left_cm,rear_us_cm,ax_mg,ay_mg,az_mg
```

## Plotting Logs

Install Python plotting dependencies:

```bash
pip install -r tools/requirements.txt
```

Generate presentation-ready plots from a Tera Term or serial log:

```bash
python tools/plot_speed_csv_for_ppt.py teraterm_log.txt --outdir graphs
```

## Not Included

Large videos, presentation files, generated logs, IDE metadata, compiled binaries, and debug outputs are intentionally excluded from this public repository.
