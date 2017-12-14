#!/usr/bin/env python3
# Copyright (c) 2017 Angel Terrones <angelterrones@gmail.com>

import argparse
import myhdl as hdl
from atik.system import Clock
from atik.system import Reset
from atik.system import Timeout
from atik.utils import Configuration
from atik.utils import run_parser
from atik.utils import run_testbench
from atik.system.interconnect import WishboneIntercon
from atik.models.Memory import WBMemorySP
from algol._csr import CoreInterrupts
from algol import CoreB


def test_core(elf, config_file):
    """Stub for pytest
    """
    args = argparse.Namespace(elf=elf, config_file=config_file, trace=False)
    core_testbench(args)


def core_testbench(args=None):
    config    = Configuration(args.config_file)
    timescale = 1e-9
    freq      = 10e6
    memsize   = config.getOption('Memory', 'size')
    tohost    = config.getOption('tohost', 'address')
    clk       = Clock(0, freq=freq, timescale=timescale)
    rst       = Reset(0, active=True, async=False)
    timeout   = Timeout(1e6)

    @hdl.block
    def tb_core():
        wb_port         = WishboneIntercon()
        core_interrupts = CoreInterrupts()
        clkgen          = clk.clk_gen()  # noqa
        tout            = timeout.timeout_gen()  # noqa
        memory          = WBMemorySP(clk_i=clk, rst_i=rst, io_port=wb_port, SIZE=memsize, ELF_FILE=args.elf, NUM_CYC=1)  # noqa
        core            = CoreB(clk_i=clk, rst_i=rst, wb_port=wb_port, core_interrupts=core_interrupts,  # noqa
                                debug=None, hart_id=0, config=config)

        @hdl.instance
        def rst_pulse():
            yield rst.pulse(300)

        @hdl.always(clk.posedge)
        def to_host_check():
            if wb_port.addr == tohost and wb_port.cyc and wb_port.stb and wb_port.we:
                if wb_port.dat_o != 1:
                    raise hdl.Error(" Test failed. to_host = {0}. Time = {1} ".format(wb_port.dat_o, hdl.now()))
                else:
                    print(" Simulation OK. Simulation time: {0} ".format(hdl.now()))
                    raise hdl.StopSimulation

        return hdl.instances()

    run_testbench(tb_core, args=args, timescale=timescale)


if __name__ == '__main__':
    args = run_parser(extra_ops=[('--elf', dict(required=True)), ('--config-file', dict(required=True))])
    core_testbench(args)
    print('[TEST-ALGOL-CORE-B] Test: Ok')

# Local Variables:
# flycheck-flake8-maximum-line-length: 200
# flycheck-flake8rc: ".flake8rc"
# End:
