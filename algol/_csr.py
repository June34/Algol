#!/usr/bin/env python
# Copyright (c) 2017 Angel Terrones <angelterrones@gmail.com>

import myhdl as hdl
from atik.utils import createSignal


class CSRAddressMap:
    SZ_ADDR     = 12
    MVENDORID   = 0xF11
    MARCHID     = 0xF12
    MIMPID      = 0xF13
    MHARTID     = 0xF14
    MSTATUS     = 0x300
    MISA        = 0x301
    MEDELEG     = 0x302
    MIDELEG     = 0x303
    MIE         = 0x304
    MTVEC       = 0x305
    MCOUNTEREN  = 0x306
    MSCRATCH    = 0x340
    MEPC        = 0x341
    MCAUSE      = 0x342
    MTVAL       = 0x343
    MIP         = 0x344
    MCYCLE      = 0xB00
    MINSTRET    = 0xB02
    MCYCLEH     = 0xB80
    MINSTRETH   = 0xB82
    #
    DCSR        = 0x7B0
    DPC         = 0x7B1
    DSCRATCH    = 0x7B2


class ExceptionCode:
    SZ_ECODE                    = 4
    E_INST_ADDR_MISALIGNED      = 0
    E_INST_ACCESS_FAULT         = 1
    E_ILLEGAL_INST              = 2
    E_BREAKPOINT                = 3
    E_LOAD_ADDR_MISALIGNED      = 4
    E_LOAD_ACCESS_FAULT         = 5
    E_STORE_AMO_ADDR_MISALIGNED = 6
    E_STORE_AMO_ACCESS_FAULT    = 7
    E_ECALL_FROM_U              = 8
    E_ECALL_FROM_S              = 9
    E_ECALL_FROM_M              = 11
    I_U_SOFTWARE                = 0
    I_S_SOFTWARE                = 1
    I_M_SOFTWARE                = 3
    I_U_TIMER                   = 4
    I_S_TIMER                   = 5
    I_M_TIMER                   = 7
    I_U_EXTERNAL                = 8
    I_S_EXTERNAL                = 9
    I_M_EXTERNAL                = 11


class CSRModes:
    SZ_MODE = 2
    PRIV_U  = 0
    PRIV_S  = 1
    PRIV_M  = 3


class CoreInterrupts:
    def __init__(self):
        self.meip = createSignal(0, 1)
        self.mtip = createSignal(0, 1)
        self.msip = createSignal(0, 1)


class CSRIO:
    def __init__(self):
        self.addr_i   = createSignal(0, CSRAddressMap.SZ_ADDR)
        self.wdat_i   = createSignal(0, 32)
        self.rdat_o   = createSignal(0, 32)
        self.r_i      = createSignal(0, 1)
        self.w_i      = createSignal(0, 1)
        self.s_i      = createSignal(0, 1)
        self.c_i      = createSignal(0, 1)
        self.system_i = createSignal(0, 1)


class CSRExceptionIO:
    def __init__(self):
        self.exception_i      = createSignal(0, 1)
        self.exception_code_i = createSignal(0, ExceptionCode.SZ_ECODE)
        self.exception_pc_i   = createSignal(0, 32)
        self.exception_dat_i  = createSignal(0, 32)
        self.next_pc_o        = createSignal(0, 32)
        self.kill_o           = createSignal(0, 1)


@hdl.block
def CSR(clk_i, rst_i, enable_i, retire_i, io, eio, core_interrupts, HART_ID, RST_ADDR, EXTENSIONS):
    assert isinstance(io, CSRIO)
    assert isinstance(eio, CSRExceptionIO)
    assert isinstance(core_interrupts, CoreInterrupts)

    # CSR registers
    mvendorid  = 0
    marchid    = 0
    mimpid     = 0
    mstatus    = createSignal(0, 32)
    mie        = createSignal(0, 32)
    mtvec      = createSignal(RST_ADDR, 32)
    mscratch   = createSignal(0, 32)
    mepc       = createSignal(0, 32)
    mcause     = createSignal(0, 32)
    mtval      = createSignal(0, 32)
    mip        = createSignal(0, 32)
    cycle      = createSignal(0, 64)
    instret    = createSignal(0, 64)
    # mstatus fields
    mpp        = createSignal(0, 2)
    mpie       = createSignal(0, 1)
    _mie       = createSignal(0, 1)
    # mie fields
    meie       = createSignal(0, 1)
    mtie       = createSignal(0, 1)
    msie       = createSignal(0, 1)
    # mcause
    _interrupt = createSignal(0, 1)
    mecode     = createSignal(0, ExceptionCode.SZ_ECODE)
    # aux signals
    priv_mode  = createSignal(CSRModes.PRIV_M, CSRModes.SZ_MODE)
    mdat       = createSignal(0, 32)
    wd_aux     = createSignal(0, 32)  # aux write data
    undef_reg  = createSignal(0, 1)
    pend_int   = createSignal(0, 32)  # pending interrupts
    priv_valid = createSignal(0, 1)   # priv level is ok. Can execute command
    wen        = createSignal(0, 1)   # is ok to update registers
    xcall      = createSignal(0, 1)
    xbreak     = createSignal(0, 1)
    xret       = createSignal(0, 1)
    ill_access = createSignal(0, 1)
    exception  = createSignal(0, 1)  #
    minterrupt = createSignal(0, 1)  #
    int_code   = createSignal(0, ExceptionCode.SZ_ECODE)  #
    no_trap    = createSignal(0, 1)

    # --------------------------------------------------------------------------
    # assignments
    @hdl.always_comb
    def assign_xfunc_proc():
        xcall.next  = enable_i and priv_valid and io.system_i and io.addr_i[3:0] == 0x00
        xbreak.next = enable_i and priv_valid and io.system_i and io.addr_i[3:0] == 0x01
        xret.next   = enable_i and priv_valid and io.system_i and io.addr_i[3:0] == 0x02

    @hdl.always_comb
    def assign1_proc():
        mstatus.next    = hdl.concat(hdl.modbv(0)[19:], mpp, hdl.modbv(0)[3:], mpie, hdl.modbv(0)[3:], _mie, hdl.modbv(0)[3:])
        mip.next        = hdl.concat(hdl.modbv(0)[20:], core_interrupts.meip, hdl.modbv(0)[3:], core_interrupts.mtip, hdl.modbv(0)[3:], core_interrupts.msip, hdl.modbv(0)[3:])
        mie.next        = hdl.concat(hdl.modbv(0)[20:], meie, hdl.modbv(0)[3:], mtie, hdl.modbv(0)[3:], msie, hdl.modbv(0)[3:])
        mcause.next     = hdl.concat(_interrupt, hdl.modbv(0)[32 - ExceptionCode.SZ_ECODE - 1:], mecode)
        priv_valid.next = priv_mode >= io.addr_i[10:8]
        exception.next  = eio.exception_i or ill_access or xcall or xbreak

    @hdl.always_comb
    def assign2_proc():
        wen.next      = priv_valid and (io.w_i or io.c_i or io.s_i) and enable_i
        no_trap.next  = not (eio.exception_i or ill_access or minterrupt)
        pend_int.next = mip & mie if (priv_mode < CSRModes.PRIV_M or (priv_mode == CSRModes.PRIV_M and _mie)) else 0

    @hdl.always_comb
    def assign3_proc():
        ill_access.next = (wen and io.addr_i[12:10] == 0b11) or \
                          (io.r_i or io.w_i or io.c_i or io.s_i) and (not priv_valid or undef_reg) or \
                          (io.system_i and not priv_valid)

    # --------------------------------------------------------------------------
    # counters
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def counters_proc():
        cycle.next   = cycle + 1
        instret.next = instret + (no_trap and retire_i)

    # --------------------------------------------------------------------------
    # interrupts
    @hdl.always_comb
    def interrupt_proc():
        minterrupt.next = enable_i and (pend_int[11] or pend_int[7] or pend_int[3])
        int_code.next   = (ExceptionCode.I_M_EXTERNAL if pend_int[11] else
                           (ExceptionCode.I_M_TIMER if pend_int[7] else
                            ExceptionCode.I_M_SOFTWARE))

    # --------------------------------------------------------------------------
    # priv mode
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def priv_mode_proc():
        if exception or minterrupt:
            priv_mode.next = CSRModes.PRIV_M
        elif xret:
            priv_mode.next = mpp

    # --------------------------------------------------------------------------
    # write
    @hdl.always_comb
    def wdata_aux_proc():
        if io.s_i:
            wd_aux.next = mdat | io.wdat_i
        elif io.c_i:
            wd_aux.next = mdat & ~io.wdat_i
        else:
            wd_aux.next = io.wdat_i

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mstatus_mode_proc():
        if exception or minterrupt:
            mpp.next  = priv_mode
            mpie.next = mstatus[priv_mode]
            _mie.next = False
        elif xret:
            mpp.next  = CSRModes.PRIV_U
            mpie.next = True
            _mie.next = mpie if mpp == CSRModes.PRIV_M else True  # TODO: check
        elif io.addr_i == CSRAddressMap.MSTATUS:
            mpp.next  = wd_aux[13:11]
            mpie.next = wd_aux[7]
            _mie.next = wd_aux[3]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mepc_proc():
        if exception or minterrupt:
            mepc.next = hdl.concat(eio.exception_pc_i[:2], False, False)
        elif wen and io.addr_i == CSRAddressMap.MEPC:
            mepc.next = hdl.concat(wd_aux[:2], False, False)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mcause_proc():
        if minterrupt:
            _interrupt.next = True
            mecode.next = int_code
        elif exception:
            _interrupt.next = False
            mecode.next = (eio.exception_code_i if eio.exception_i else
                           (ExceptionCode.E_ECALL_FROM_U + priv_mode if xcall else
                            (ExceptionCode.E_BREAKPOINT if xbreak else
                             ExceptionCode.E_ILLEGAL_INST)))
        elif wen and io.addr_i == CSRAddressMap.MCAUSE:
            _interrupt.next = wd_aux[31]
            mecode.next     = wd_aux[ExceptionCode.SZ_ECODE:0]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mtval_proc():
        if exception:
            mtval.next = eio.exception_dat_i
        elif wen and io.addr_i == CSRAddressMap.MTVAL:
            mtval.next = wd_aux

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mie_w_proc():
        if io.addr_i == CSRAddressMap.MIE:
            meie.next = wd_aux[11]
            mtie.next = wd_aux[7]
            msie.next = wd_aux[3]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def write_proc():
        if wen:
            if io.addr_i == CSRAddressMap.MTVEC:
                mtvec.next = wd_aux  # force bits 0,1 to zero?
            elif io.addr_i == CSRAddressMap.MSCRATCH:
                mscratch.next = wd_aux

    # --------------------------------------------------------------------------
    # read
    @hdl.always_comb
    def csr_read_proc():
        undef_reg.next = False
        if io.addr_i == CSRAddressMap.MISA:
            mdat.next = hdl.concat(hdl.modbv(1)[2:], hdl.modbv(0)[4:], hdl.modbv(EXTENSIONS)[26:])
        elif io.addr_i == CSRAddressMap.MHARTID:
            mdat.next = hdl.modbv(HART_ID)[32:]
        elif io.addr_i == CSRAddressMap.MVENDORID:
            mdat.next = mvendorid
        elif io.addr_i == CSRAddressMap.MARCHID:
            mdat.next = marchid
        elif io.addr_i == CSRAddressMap.MIMPID:
            mdat.next = mimpid
        elif io.addr_i == CSRAddressMap.MSTATUS:
            mdat.next = mstatus
        elif io.addr_i == CSRAddressMap.MIE:
            mdat.next = mie
        elif io.addr_i == CSRAddressMap.MTVEC:
            mdat.next = mtvec
        elif io.addr_i == CSRAddressMap.MSCRATCH:
            mdat.next = mscratch
        elif io.addr_i == CSRAddressMap.MEPC:
            mdat.next = mepc
        elif io.addr_i == CSRAddressMap.MCAUSE:
            mdat.next = mcause
        elif io.addr_i == CSRAddressMap.MTVAL:
            mdat.next = mtval
        elif io.addr_i == CSRAddressMap.MIP:
            mdat.next = mip
        elif io.addr_i == CSRAddressMap.MCYCLE:
            mdat.next = cycle[32:0]
        elif io.addr_i == CSRAddressMap.MINSTRET:
            mdat.next = instret[32:0]
        elif io.addr_i == CSRAddressMap.MCYCLEH:
            mdat.next = cycle[64:32]
        elif io.addr_i == CSRAddressMap.MINSTRETH:
            mdat.next = instret[64:32]
        else:
            mdat.next = 0
            undef_reg.next = True

    # --------------------------------------------------------------------------
    # IO assignments
    @hdl.always_comb
    def io_assign_proc():
        io.rdat_o.next     = mdat
        eio.next_pc_o.next = mtvec if exception or minterrupt else mepc
        eio.kill_o.next    = exception or minterrupt or xret

    return hdl.instances()

# Local Variables:
# flycheck-flake8-maximum-line-length: 200
# flycheck-flake8rc: ".flake8rc"
# End:
