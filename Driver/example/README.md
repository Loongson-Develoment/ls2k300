# Driver Examples

该目录用于存放 `Driver/inc` 与 `Driver/src` 的独立示例程序，结构参考 `library/example`。

当前包含：

- `st7735/main.c`：ICST7735 屏幕驱动示例（硬件 SPI）

## 目录说明

- `st7735/CMakeLists.txt`：单示例构建脚本
- `st7735/build.sh`：单示例一键构建
- `CMakeLists.txt`：示例总入口（可扩展多个子示例）
- `build.sh`：总入口一键构建

## 构建

构建全部 Driver 示例：

```bash
cd Driver/example
./build.sh
```

构建单个 ST7735 示例：

```bash
cd Driver/example/st7735
./build.sh
```

## 运行

```bash
cd Driver/example/st7735
sudo ./example_st7735
```

或在 `Driver/example` 目录：

```bash
sudo ./example_st7735
```

## 说明

- 本示例依赖 `/dev/mem`，通常需要 root 权限。
- SPI2 的时钟/数据/片选引脚由底层 `LS2K0300_SPI` 驱动固定映射。
- 示例中的 `DC/RST/BL` 引脚仅作参考，请按你的实际接线修改 `st7735/main.c` 宏定义。
