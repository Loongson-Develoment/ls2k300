# LS2K0300 Library Examples

This directory contains standalone examples for `library/inc` and `library/src`.

Each test case now has its own folder and `main.c`:

- `signal/main.c`
- `gpio/main.c`
- `adc/main.c`
- `pwm/main.c`
- `uart/main.c`
- `i2c/main.c`
- `spi/main.c`
- `soft_spi/main.c`
- `atim_pwm/main.c`
- `gtim_pwm/main.c`
- `encoder/main.c`
- `timer/main.c`
- `canfd/main.c`

## Build

```bash
cd library/example
./build.sh
```

Or with CMake directly:

```bash
cmake -S . -B build
cmake --build build -j8
```

## Run

After build, executables are under `library/example` and named by source file:

```bash
./example_gpio
./example_uart
./example_canfd
```

`example_uart` is configured as `EXCLUDE_FROM_ALL` and can be built manually:

```bash
cmake --build build --target example_uart
```

## Notes

- Most register-based modules need `/dev/mem` access, usually with root privileges.
- `example_canfd` configures `can0`, usually requiring root privileges.
- Default pins are only references. Adjust according to your board wiring.
