# LS2K0300 Library Examples

This directory contains usage examples for the `library/inc` and `library/src` APIs.

## Files

- `ls2k0300_library_examples.c`
  - Provides one function per module, for example:
    - `ls2k0300_example_gpio`
    - `ls2k0300_example_adc`
    - `ls2k0300_example_pwm`
    - `ls2k0300_example_uart`
    - `ls2k0300_example_i2c`
    - `ls2k0300_example_spi`
    - `ls2k0300_example_soft_spi`
    - `ls2k0300_example_atim_pwm`
    - `ls2k0300_example_gtim_pwm`
    - `ls2k0300_example_encoder`
    - `ls2k0300_example_timer`
    - `ls2k0300_example_canfd`

## How To Use

1. As snippets:
   - Include `LS2K0300_DRV_INC.h` in your app.
   - Copy the module function you need into your own source file.

2. As a standalone example suite:
   - Build this file with macro `LS2K0300_EXAMPLE_RUN_ALL` to enable `main`.
   - Then run and check PASS/FAIL/SKIP output.

Example command:

```bash
/usr/local/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc \
  -std=c11 \
  -I./library/inc \
  ./library/example/ls2k0300_library_examples.c \
  ./library/src/*.c \
  -lpthread \
  -DLS2K0300_EXAMPLE_RUN_ALL \
  -o ./library/example/ls2k0300_examples
```

## Notes

- Most register-based modules require access to `/dev/mem`, usually with root privileges.
- CANFD setup in the example configures `can0` and also usually requires root privileges.
- Pin choices in the examples are only reference defaults. Adjust pins/mux according to your board wiring.
