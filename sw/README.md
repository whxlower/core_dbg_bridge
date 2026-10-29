# uart2axi 主机端工具

UART -> AXI 调试桥（本仓库为 ultraembedded/dbg_bridge 的非官方修改版）的 PC 端软件。

| 文件 | 说明 |
|---|---|
| `uart2axi_gui.py` | 图形界面：串口扫描 / 寄存器读写 / GPIO / 批量读写 / 通信日志 |
| `peek.py` / `poke.py` | 单次读 / 写（上游脚本，未修改） |
| `dump.py` / `load_bin.py` | 内存导出 / 二进制加载（上游脚本，未修改） |
| `bus_interface.py` | UART / socket 后端封装（上游脚本，未修改） |
| `build_exe.ps1` | 用 PyInstaller 打包 Windows 可执行文件 |

线上协议与上游一致：
写命令 `0x10, len, addr[31:0] (MSB first), data[len] (LSB first)`；
读命令 `0x11, len, addr[31:0]`，回复 `len` 字节。

## 运行

```powershell
uv sync
uv run python uart2axi_gui.py
```

## 打包成 exe

```powershell
.\build_exe.ps1              # 单文件：dist\uart2axi_gui.exe
.\build_exe.ps1 -OneDir      # 目录版：dist\uart2axi_gui\
```

## 许可

本目录与整个仓库均以 **LGPL-2.1-or-later** 发布，详见仓库根目录的
[`LICENSE`](../LICENSE) 与 [`NOTICE`](../NOTICE)（其中列出了上游文件、
被修改的文件以及本 fork 新增的文件）。
