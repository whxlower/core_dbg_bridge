> **Unofficial fork.** This repository is a modified fork of
> [ultraembedded/dbg_bridge](https://github.com/ultraembedded/dbg_bridge)
> (baseline commit `3ea8f99`, LGPL-2.1-or-later), maintained by
> [whxlower](https://github.com/whxlower). It is not affiliated with, nor
> endorsed by, Ultra-Embedded.com. The fork adds AXI4 port naming and
> attributes, byte-lane write strobes and a GUI tool under `sw/`.
> See [NOTICE](NOTICE) for the full list of modified and added files.
> All files in this repository are licensed under LGPL-2.1-or-later;
> see [LICENSE](LICENSE).

### UART -> AXI Debug Bridge

Github:   [http://github.com/ultraembedded/cores](https://github.com/ultraembedded/cores/tree/master/dbg_bridge)

This component provides a bridge from a standard UART interface (8N1) to a AXI4 bus master & GPIO interface.  
This can be very useful for FPGA dev boards featuring a FTDI UART interface where loading memories, peeking, poking SoC state is required.

##### Testing

Used extensively on various Xilinx FPGAs over the years.

##### Configuration
* CLK_FREQ - Clock (clk_i) frequency (in Hz).
* UART_SPEED - UART baud rate (bps)
* AXI_ID - AXI ID to be used for transactions

##### Software
Included python based utils provide peek and poke access, plus binary load / dump support.

Examples:
```
# Read a memory location (0x0)
./sw/peek.py -d /dev/ttyUSB1 -b 115200 -a 0x0

# Write a memory word (0x0 = 0x12345678)
./sw/poke.py -d /dev/ttyUSB1 -b 115200 -a 0x0 -v 0x12345678
```

##### License

This repository is an unofficial, modified fork of
[ultraembedded/dbg_bridge](https://github.com/ultraembedded/dbg_bridge),
Copyright (C) 2017-2019 Ultra-Embedded.com.

Modifications Copyright (C) 2026 whx <253616211@qq.com>, made on 2026-10-29.

All files are distributed under the **GNU Lesser General Public License,
version 2.1 or (at your option) any later version** &mdash; see [LICENSE](LICENSE).
The upstream files, the modified files, the added files and the exact changes
are listed in [NOTICE](NOTICE).

There is NO WARRANTY for this software. Please report issues to this
repository, not to the upstream author.
