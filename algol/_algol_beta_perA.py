#!/usr/bin/env python3
# Copyright (c) 2017 Angel Terrones (<angelterrones@gmail.com>)

import myhdl as hdl


@hdl.block
def CoreA(clk_i, rst_i, iport, dport, HART_ID=0):
    raise NotImplemented
    return hdl.instances()

# Local Variables:
# flycheck-flake8-maximum-line-length: 300
# flycheck-flake8rc: ".flake8rc"
# End:
