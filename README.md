![logo](documentation/img/logo.png)

ALGOL - A RISC-V CPU
====================

Algol is a set of CPU cores that implement the [RISC-V RV32I Instruction Set](http://riscv.org/).

Algol is free and open hardware licensed under the [MIT license](https://en.wikipedia.org/wiki/MIT_License).

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ALGOL - A RISC-V CPU](#algol---a-risc-v-cpu)
    - [Dependencies](#dependencies)
    - [Processor details](#processor-details)
        - [Core A: Beta Persei Aa1.](#core-a-beta-persei-aa1)
        - [Core B: Beta Persei Aa2.](#core-b-beta-persei-aa2)
    - [Software Details](#software-details)
    - [Directory Layout](#directory-layout)
    - [Validation](#validation)
        - [Compile assembly tests](#compile-assembly-tests)
        - [Validate cores](#validate-cores)
    - [RISC-V toolchain](#risc-v-toolchain)
    - [TODO](#todo)
    - [License](#license)

<!-- markdown-toc end -->
Dependencies
------------
- Python 3.
- pytest.
- [myhdl](https://github.com/myhdl/myhdl).
- [Icarus Verilog](http://iverilog.icarus.com) for Cosimulation.
- [Atik](https://github.com/AngelTerrones/Atik).
- RISC-V toolchain, to compile the validation tests.

Processor details
-----------------
### Core A: Beta Persei Aa1.
This core is under development.

### Core B: Beta Persei Aa2.
- RISC-V RV32I ISA.
- Support for the Machine and User [privilege modes](https://riscv.org/specifications/privileged-isa/)
- Multi-cycle datapath.
- Single memory port using the [Wishbone B4](https://www.ohwr.org/attachments/179/wbspec_b4.pdf) Interface.
- Designed completely in python using [MyHDL](http://myhdl.org/).

Software Details
----------------
- Simulation and Cosimulation done in python using [MyHDL](http://myhdl.org/) and [Icarus Verilog](http://iverilog.icarus.com).
- [Toolchain](http://riscv.org/software-tools/) using gcc.
- [Validation suit](http://riscv.org/software-tools/riscv-tests/) written in assembly

Directory Layout
----------------
- `README.md`: This file
- `algol`: Python/MyHDL files describing the CPU.
- `documentation`: LaTeX source files for the CPU manuals (TODO).
- `tests`: Test environment for the Algol CPU.
    - `core`: Testbench for the CPU validation.
    - `riscv_test`: Basic instruction-level tests. Taken from [riscv-tests](http://riscv.org/software-tools/riscv-tests/) (git rev b747a10**

Validation
----------
### Compile assembly tests
To compile the RISC-V assembly tests:

> $ cd $(ALGOL's folder)/tests/riscv_test/
> $ make

### Validate cores
To validate the cores, run `pytest` in the root folder (No VCD dumps):

> $ pytest -v

To validate using a single ISA test using the MyHDL simulator:

> $ python3 tests/core/test_core_b.py --elf tests/riscv_test/[elf file] --config-file tests/core/[ini file]

To validate using a single ISA test using cosimulation:

> $ python3 tests/core/test_cosim_core_b.py --elf tests/riscv_test/[elf file] --config-file tests/core/[ini file]

To enable dump of VCD files, add the `--trace` flag.

RISC-V toolchain
----------------
The easy way to get the toolchain is to download a pre-compiled version from the
[GNU MCU Eclipse](https://gnu-mcu-eclipse.github.io/) project.

The version used to validate this core is the [Embedded GCC v7.2.0-1-20171109](https://gnu-mcu-eclipse.github.io/blog/2017/11/09/riscv-none-gcc-v7-2-0-1-20171109-released/)

TODO
----
- RV32M ISA.
- Debug module.
- Core A: Beta Persei Aa1

License
-------
Copyright (c) 2017 Angel Terrones (<angelterrones@gmail.com>).

Release under the [MIT License](MITlicense.md).
