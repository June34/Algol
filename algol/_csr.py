#!/usr/bin/env python
# Copyright (c) 2017 Angel Terrones <angelterrones@gmail.com>

import myhdl as hdl
from atik.utils import createSignal
from atik.utils import Configuration


class CSRAddressMap:
    SZ_ADDR     = 12
    CYCLE       = 0xC00
    INSTRET     = 0xC02
    CYCLEH      = 0xC80
    INSTRETH    = 0xC82
    #
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
        self.addr0_i  = createSignal(0, CSRAddressMap.SZ_ADDR)
        self.addr1_i  = createSignal(0, CSRAddressMap.SZ_ADDR)
        self.addr2_i  = createSignal(0, CSRAddressMap.SZ_ADDR)
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
def CSR(clk_i, rst_i, retire_i, enable_i, ack_o, io, eio, core_interrupts, hart_id, config):
    assert isinstance(io, CSRIO)
    assert isinstance(eio, CSRExceptionIO)
    assert isinstance(core_interrupts, CoreInterrupts)
    assert isinstance(config, Configuration)

    # read configuration
    extension    = int(config.getOption('ISA', 'extension'), 2)
    rst_addr     = config.getOption('Core', 'start_address')
    enableU      = extension & (1 << 20)
    enableS      = extension & (1 << 18)
    assert not enableS, "[CSR] Error: Supervisor move is not supported."
    assert enableU, "[CSR] Error: User mode is mandatory. Please, enable User mode in the configuration file."
    # CSR registers
    mstatus      = createSignal(0, 32)
    mie          = createSignal(0, 32)
    mtvec        = createSignal(rst_addr, 32)
    mscratch     = createSignal(0, 32)
    mepc         = createSignal(0, 32)
    mcause       = createSignal(0, 32)
    mtval        = createSignal(0, 32)
    mip          = createSignal(0, 32)
    cycle        = createSignal(0, 64)
    instret      = createSignal(0, 64)
    # mstatus fields
    mpp          = createSignal(0, 2)
    mpie         = createSignal(0, 1)
    _mie         = createSignal(0, 1)
    # mie fields
    meie         = createSignal(0, 1)
    mtie         = createSignal(0, 1)
    msie         = createSignal(0, 1)
    # mcause
    _interrupt   = createSignal(0, 1)
    mecode       = createSignal(0, ExceptionCode.SZ_ECODE)
    _mecode      = createSignal(0, ExceptionCode.SZ_ECODE)
    # aux signals
    priv_mode    = createSignal(CSRModes.PRIV_M, CSRModes.SZ_MODE)
    mdat         = createSignal(0, 32)
    wd_aux       = createSignal(0, 32)  # aux write data
    undef_reg    = createSignal(0, 1)
    pend_int     = createSignal(0, 32)  # pending interrupts
    priv_valid   = createSignal(0, 1)   # priv level is ok. Can execute command
    wen          = createSignal(0, 1)   # is ok to update registers
    xcall        = createSignal(0, 1)
    xbreak       = createSignal(0, 1)
    xret         = createSignal(0, 1)
    ill_access   = createSignal(0, 1)
    exception    = createSignal(0, 1)  #
    minterrupt   = createSignal(0, 1)  #
    int_code     = createSignal(0, ExceptionCode.SZ_ECODE)  #
    no_trap      = createSignal(0, 1)
    #
    is_misa      = createSignal(0, 1)
    is_mhartid   = createSignal(0, 1)
    is_mvendorid = createSignal(0, 1)
    is_marchid   = createSignal(0, 1)
    is_mimpid    = createSignal(0, 1)
    is_mstatus   = createSignal(0, 1)
    is_mie       = createSignal(0, 1)
    is_mtvec     = createSignal(0, 1)
    is_mscratch  = createSignal(0, 1)
    is_mepc      = createSignal(0, 1)
    is_mcause    = createSignal(0, 1)
    is_mtval     = createSignal(0, 1)
    is_mip       = createSignal(0, 1)
    is_cycle     = createSignal(0, 1)
    is_instret   = createSignal(0, 1)
    is_cycleh    = createSignal(0, 1)
    is_instreth  = createSignal(0, 1)
    #
    delay        = createSignal(0, 4)

    # --------------------------------------------------------------------------
    # Delay for CSR operations
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def delay_proc():
        if enable_i and not (exception or minterrupt or xret):
            delay.next = hdl.concat(delay[3:0], enable_i and delay == 0)
        else:
            delay.next = 0

    @hdl.always_comb
    def ack_assign_proc():
        ack_o.next = delay[3]

    # --------------------------------------------------------------------------
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def check_address_proc():
        is_misa.next      = io.addr1_i == CSRAddressMap.MISA
        is_mhartid.next   = io.addr1_i == CSRAddressMap.MHARTID
        is_mstatus.next   = io.addr1_i == CSRAddressMap.MSTATUS
        is_mie.next       = io.addr1_i == CSRAddressMap.MIE
        is_mtvec.next     = io.addr1_i == CSRAddressMap.MTVEC
        is_mscratch.next  = io.addr1_i == CSRAddressMap.MSCRATCH
        is_mepc.next      = io.addr1_i == CSRAddressMap.MEPC
        is_mcause.next    = io.addr1_i == CSRAddressMap.MCAUSE
        is_mtval.next     = io.addr1_i == CSRAddressMap.MTVAL
        is_mip.next       = io.addr2_i == CSRAddressMap.MIP
        is_cycle.next     = io.addr2_i == CSRAddressMap.CYCLE or io.addr2_i == CSRAddressMap.CYCLE
        is_instret.next   = io.addr2_i == CSRAddressMap.INSTRET or io.addr2_i == CSRAddressMap.INSTRET
        is_cycleh.next    = io.addr2_i == CSRAddressMap.CYCLEH or io.addr2_i == CSRAddressMap.CYCLE
        is_instreth.next  = io.addr2_i == CSRAddressMap.INSTRETH or io.addr2_i == CSRAddressMap.INSTRETH
        is_mvendorid.next = io.addr2_i == CSRAddressMap.MVENDORID
        is_marchid.next   = io.addr2_i == CSRAddressMap.MARCHID
        is_mimpid.next    = io.addr2_i == CSRAddressMap.MIMPID

    # --------------------------------------------------------------------------
    @hdl.always_comb
    def mregisters_assign_proc():
        mstatus.next = hdl.concat(hdl.modbv(0)[19:], mpp, hdl.modbv(0)[3:], mpie, hdl.modbv(0)[3:], _mie, hdl.modbv(0)[3:])
        mip.next     = hdl.concat(hdl.modbv(0)[20:], core_interrupts.meip, hdl.modbv(0)[3:], core_interrupts.mtip, hdl.modbv(0)[3:], core_interrupts.msip, hdl.modbv(0)[3:])
        mie.next     = hdl.concat(hdl.modbv(0)[20:], meie, hdl.modbv(0)[3:], mtie, hdl.modbv(0)[3:], msie, hdl.modbv(0)[3:])
        mcause.next  = hdl.concat(_interrupt, hdl.modbv(0)[32 - ExceptionCode.SZ_ECODE - 1:], mecode)

    @hdl.always_comb
    def check_priv_mode_proc():
        priv_valid.next = priv_mode >= io.addr0_i[10:8]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def assign_xfunc_proc():
        xcall.next  = enable_i and priv_valid and io.system_i and io.addr0_i[3:0] == 0x00
        xbreak.next = enable_i and priv_valid and io.system_i and io.addr0_i[3:0] == 0x01
        xret.next   = enable_i and priv_valid and io.system_i and io.addr0_i[3:0] == 0x02

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def latch_wen_proc():
        wen.next = priv_valid and (io.w_i or io.c_i or io.s_i) and enable_i

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def check_valid_access_proc():
        ill_access.next = (wen and io.addr0_i[12:10] == 0b11) or \
                          (io.r_i or io.w_i or io.c_i or io.s_i) and (not priv_valid or undef_reg) or \
                          (io.system_i and not priv_valid)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def check_pending_interrupts_proc():
        pend_int.next = mip & mie if (priv_mode < CSRModes.PRIV_M or (priv_mode == CSRModes.PRIV_M and _mie)) else 0

    # --------------------------------------------------------------------------
    # counters
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def counters_proc():
        cycle.next   = cycle + 1
        instret.next = instret + (no_trap and (retire_i or delay[3]))

    # --------------------------------------------------------------------------
    # interrupts
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def exception_proc():
        exception.next = eio.exception_i or ill_access or xcall or xbreak
        no_trap.next   = not (eio.exception_i or ill_access or minterrupt)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def interrupt_proc():
        minterrupt.next = enable_i and (pend_int[11] or pend_int[7] or pend_int[3])
        if pend_int[11]:
            int_code.next = ExceptionCode.I_M_EXTERNAL
        elif pend_int[7]:
            int_code.next = ExceptionCode.I_M_TIMER
        else:
            int_code.next = ExceptionCode.I_M_SOFTWARE

    # --------------------------------------------------------------------------
    # priv mode
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def priv_mode_proc():
        if (exception or minterrupt) and enable_i:
            priv_mode.next = CSRModes.PRIV_M
        elif xret and enable_i:
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
        if (exception or minterrupt) and enable_i:
            mpp.next  = priv_mode
            mpie.next = _mie
            _mie.next = False
        elif xret and enable_i:
            mpp.next  = CSRModes.PRIV_U
            mpie.next = True
            _mie.next = mpie
        elif wen and delay[3] and is_mstatus:
            mpp.next  = hdl.concat(wd_aux[11] or wd_aux[12], wd_aux[11] or wd_aux[12])  # 00 or 11. No other values (for now)
            mpie.next = wd_aux[7]
            _mie.next = wd_aux[3]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mepc_proc():
        if (exception or minterrupt) and enable_i:
            mepc.next = hdl.concat(eio.exception_pc_i[:2], False, False)
        elif wen and delay[3] and is_mepc:
            mepc.next = hdl.concat(wd_aux[:2], False, False)

    @hdl.always_comb
    def mcode_assign_proc():
        if eio.exception_i:
            _mecode.next = eio.exception_code_i
        elif xcall:
            _mecode.next = ExceptionCode.E_ECALL_FROM_U + priv_mode
        elif xbreak:
            _mecode.next = ExceptionCode.E_BREAKPOINT
        else:
            _mecode.next = ExceptionCode.E_ILLEGAL_INST

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mcause_proc():
        if minterrupt and enable_i:
            _interrupt.next = True
            mecode.next = int_code
        elif exception and enable_i:
            _interrupt.next = False
            mecode.next     = _mecode
        elif wen and delay[3] and is_mcause:
            _interrupt.next = wd_aux[31]
            mecode.next     = wd_aux[ExceptionCode.SZ_ECODE:0]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mtval_proc():
        if exception and enable_i:
            mtval.next = eio.exception_dat_i  # TODO: check if this is the final value (because of internal exc)
        elif wen and delay[3] and is_mtval:
            mtval.next = wd_aux

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mie_w_proc():
        if wen and delay[3] and is_mie:
            meie.next = wd_aux[11]
            mtie.next = wd_aux[7]
            msie.next = wd_aux[3]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def write_proc():
        if wen and delay[3]:
            if is_mtvec:
                mtvec.next = wd_aux  # force bits 0,1 to zero?
            elif is_mscratch:
                mscratch.next = wd_aux

    # --------------------------------------------------------------------------
    # read
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def csr_read_proc():
        undef_reg.next = False
        if is_misa:
            mdat.next = hdl.concat(hdl.modbv(1)[2:], hdl.modbv(0)[4:], hdl.modbv(extension)[26:])
        elif is_mhartid:
            mdat.next = hdl.modbv(hart_id)[32:]
        elif is_mvendorid or is_marchid or is_mimpid:
            mdat.next = 0
        elif is_mstatus:
            mdat.next = mstatus
        elif is_mie:
            mdat.next = mie
        elif is_mtvec:
            mdat.next = mtvec
        elif is_mscratch:
            mdat.next = mscratch
        elif is_mepc:
            mdat.next = mepc
        elif is_mcause:
            mdat.next = mcause
        elif is_mtval:
            mdat.next = mtval
        elif is_mip:
            mdat.next = mip
        elif is_cycle:
            mdat.next = cycle[32:0]
        elif is_instret:
            mdat.next = instret[32:0]
        elif is_cycleh:
            mdat.next = cycle[64:32]
        elif is_instreth:
            mdat.next = instret[64:32]
        else:
            mdat.next = 0
            undef_reg.next = enable_i and delay != 0

    # --------------------------------------------------------------------------
    # IO assignments
    @hdl.always_comb
    def io_assign_proc():
        io.rdat_o.next     = mdat
        eio.next_pc_o.next = mtvec if exception or minterrupt else mepc
        eio.kill_o.next    = enable_i and (exception or minterrupt or xret)

    return hdl.instances()

# Local Variables:
# flycheck-flake8-maximum-line-length: 200
# flycheck-flake8rc: ".flake8rc"
# End:
