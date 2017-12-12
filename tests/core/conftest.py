#!/usr/bin/env python
# Copyright (c) 2017 Angel Terrones (<angelterrones@gmail.com>)

import glob


def pytest_generate_tests(metafunc):
    list_elf = glob.glob("tests/riscv_test/rv32ui-p-*.elf")
    # list_elf = list_elf + glob.glob("tests/riscv-tests/elf/rv32um-p-*")
    # list_elf = list_elf + glob.glob("tests/riscv-tests/elf/rv32mi-p-[!b]*")
    # list_elf = list_elf + glob.glob("tests/riscv-tests/elf/rv32ua-p-*")
    metafunc.parametrize('elf', list_elf)

# Local Variables:
# flycheck-flake8-maximum-line-length: 120
# flycheck-flake8rc: ".flake8rc"
# End:
