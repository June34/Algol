#!/usr/bin/env python3
# Copyright (c) 2017 Angel Terrones <angelterrones@gmail.com>

import myhdl as hdl
from atik.utils import createSignal
from atik.utils import Configuration
from atik.system.interconnect import WishboneIntercon
from atik.system.interconnect import WishboneMaster
from algol._instructions import Opcodes
from algol._instructions import BranchFunct3
from algol._instructions import LoadFunct3
from algol._instructions import StoreFunct3
from algol._instructions import ArithmeticFunct3
from algol._instructions import SystemFunct3
from algol._csr import ExceptionCode
from algol._csr import CoreInterrupts
from algol._csr import CSRIO
from algol._csr import CSRExceptionIO
from algol._csr import CSR


@hdl.block
def CoreB(clk_i, rst_i, wb_port, core_interrupts, debug, hart_id, config):
    """Multi-cycle RV32I Core
    """
    assert isinstance(config, Configuration), "[Algol Core B] Error: config data mustbe of type Configuration"
    assert isinstance(wb_port, WishboneIntercon), "[Algol Core B] Error: wb_port must be of type WishboneIntercon."
    assert isinstance(core_interrupts, CoreInterrupts), "[Algol Core B] Error: core_interrupts must be of type CoreInterrupts"
    assert hart_id >= 0, "[Algol Core B] Error: HART_ID must be >= 0"

    rst_addr     = config.getOption('Core', 'start_address')
    # FSM
    state_t      = hdl.enum('FETCH', 'DECODE', 'EX', 'MEM', 'WB')
    state        = hdl.Signal(state_t.FETCH)
    # WB signals
    wb_addr      = createSignal(0, 32)
    wb_rw        = createSignal(0, 1)
    wb_en        = createSignal(0, 1)
    wbm          = WishboneMaster(wb_port)
    wb           = wbm.wb_master(clk_i=clk_i, rst_i=rst_i, rw_i=wb_rw, en_i=wb_en)  # noqa
    # Fetch
    pc           = createSignal(rst_addr, 32)
    instruction  = createSignal(0x13, 32)  # default value = nop instruction
    opcode       = instruction(7, 0)
    rd           = instruction(12, 7)
    funct3       = instruction(15, 12)
    funct7       = instruction(32, 25)
    rs1_f        = instruction(20, 15)
    rs2_f        = instruction(25, 20)
    imm12        = instruction(32, 20)
    imm_i        = hdl.ConcatSignal(*[instruction(31) for _ in range(21)], instruction(31, 20))
    imm_s        = hdl.ConcatSignal(*[instruction(31) for _ in range(21)], instruction(31, 25), instruction(12, 7))
    imm_b        = hdl.ConcatSignal(*[instruction(31) for _ in range(20)], instruction(7), instruction(31, 25), instruction(12, 8), False)
    imm_u        = hdl.ConcatSignal(instruction(32, 12), hdl.modbv(0, _nrbits=12))
    imm_j        = hdl.ConcatSignal(*(instruction(31) for _ in range(12)), instruction(20, 12), instruction(20), instruction(31, 21), False)
    # decoder
    inst_lui     = createSignal(0, 1)
    inst_auipc   = createSignal(0, 1)
    inst_jal     = createSignal(0, 1)
    inst_jalr    = createSignal(0, 1)
    inst_beq     = createSignal(0, 1)
    inst_bne     = createSignal(0, 1)
    inst_blt     = createSignal(0, 1)
    inst_bge     = createSignal(0, 1)
    inst_bltu    = createSignal(0, 1)
    inst_bgeu    = createSignal(0, 1)
    inst_lb      = createSignal(0, 1)
    inst_lh      = createSignal(0, 1)
    inst_lw      = createSignal(0, 1)
    inst_lbu     = createSignal(0, 1)
    inst_lhu     = createSignal(0, 1)
    inst_sb      = createSignal(0, 1)
    inst_sh      = createSignal(0, 1)
    inst_sw      = createSignal(0, 1)
    inst_addi    = createSignal(0, 1)
    inst_slti    = createSignal(0, 1)
    inst_sltiu   = createSignal(0, 1)
    inst_xori    = createSignal(0, 1)
    inst_ori     = createSignal(0, 1)
    inst_andi    = createSignal(0, 1)
    inst_slli    = createSignal(0, 1)
    inst_srli    = createSignal(0, 1)
    inst_srai    = createSignal(0, 1)
    inst_add     = createSignal(0, 1)
    inst_sub     = createSignal(0, 1)
    inst_sll     = createSignal(0, 1)
    inst_slt     = createSignal(0, 1)
    inst_sltu    = createSignal(0, 1)
    inst_xor     = createSignal(0, 1)
    inst_srl     = createSignal(0, 1)
    inst_sra     = createSignal(0, 1)
    inst_or      = createSignal(0, 1)
    inst_and     = createSignal(0, 1)
    inst_fence   = createSignal(0, 1)
    inst_fencei  = createSignal(0, 1)
    inst_csrrw   = createSignal(0, 1)
    inst_csrrs   = createSignal(0, 1)
    inst_csrrc   = createSignal(0, 1)
    inst_csrrwi  = createSignal(0, 1)
    inst_csrrsi  = createSignal(0, 1)
    inst_csrrci  = createSignal(0, 1)
    inst_system  = createSignal(0, 1)
    is_j         = createSignal(0, 1)
    is_b         = createSignal(0, 1)
    is_l         = createSignal(0, 1)
    is_s         = createSignal(0, 1)
    is_alu       = createSignal(0, 1)
    is_csr       = createSignal(0, 1)
    is_eq        = createSignal(0, 1)
    is_lt        = createSignal(0, 1)
    is_ltu       = createSignal(0, 1)
    take_branch  = createSignal(0, 1)
    fetch_misa   = createSignal(0, 1)
    fetch_fault  = createSignal(0, 1)
    invalid_inst = createSignal(0, 1)
    load_misa    = createSignal(0, 1)
    load_fault   = createSignal(0, 1)
    store_misa   = createSignal(0, 1)
    store_fault  = createSignal(0, 1)
    exception    = createSignal(0, 1)
    # Register file
    rs1          = createSignal(0, 32)
    rs2          = createSignal(0, 32)
    rs1_d        = createSignal(0, 32)
    rs2_d        = createSignal(0, 32)
    rf_wa        = createSignal(0, 5)
    rf_we        = createSignal(0, 1)
    rf_wd        = createSignal(0, 32)
    regfile      = [createSignal(0, 32) for _ in range(32)]
    # Branch/jump
    pc_target    = createSignal(0, 32)
    # CSR
    enable_csr   = createSignal(0, 1)
    last_cycle   = createSignal(0, 1)
    csr_io       = CSRIO()
    csr_eio      = CSRExceptionIO()
    csr          = CSR(clk_i=clk_i, rst_i=rst_i, retire_i=last_cycle, enable_i=enable_csr,  # noqa
                       io=csr_io, eio=csr_eio, core_interrupts=core_interrupts, hart_id=hart_id, config=config)
    # ALU
    alu_a        = createSignal(0, 32)
    alu_b        = createSignal(0, 32)
    alu_out      = createSignal(0, 32)
    # memory
    mdat_b       = hdl.ConcatSignal(rs2_d(8, 0), rs2_d(8, 0), rs2_d(8, 0), rs2_d(8, 0))
    mdat_h       = hdl.ConcatSignal(rs2_d(16, 0), rs2_d(16, 0))
    mdat_o       = createSignal(0, 32)
    mdat_i       = createSignal(0, 32)

    # --------------------------------------------------------------------------
    # RF
    @hdl.always_comb
    def rsx_select_proc():
        rs1.next = rs1_f
        rs2.next = rs2_f
        if state == state_t.FETCH:
            rs1.next = wbm.dat_i[20:15]
            rs2.next = wbm.dat_i[25:20]

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def rf_read_proc():
        rs1_d.next = regfile[rs1] if rs1 != 0 else 0
        rs2_d.next = regfile[rs2] if rs2 != 0 else 0

    @hdl.always(clk_i.posedge)
    def rf_write_proc():
        if rf_we and rf_wa != 0:
            regfile[rf_wa].next = rf_wd

    # --------------------------------------------------------------------------
    # decoder
    @hdl.always_comb
    def decoder_proc():
        inst_lui.next    = opcode == Opcodes.RV32_LUI
        inst_auipc.next  = opcode == Opcodes.RV32_AUIPC
        #
        inst_jal.next    = opcode == Opcodes.RV32_JAL
        inst_jalr.next   = opcode == Opcodes.RV32_JALR
        #
        inst_beq.next    = opcode == Opcodes.RV32_BRANCH and funct3 == BranchFunct3.BEQ
        inst_bne.next    = opcode == Opcodes.RV32_BRANCH and funct3 == BranchFunct3.BNE
        inst_blt.next    = opcode == Opcodes.RV32_BRANCH and funct3 == BranchFunct3.BLT
        inst_bge.next    = opcode == Opcodes.RV32_BRANCH and funct3 == BranchFunct3.BGE
        inst_bltu.next   = opcode == Opcodes.RV32_BRANCH and funct3 == BranchFunct3.BLTU
        inst_bgeu.next   = opcode == Opcodes.RV32_BRANCH and funct3 == BranchFunct3.BGEU
        #
        inst_lb.next     = opcode == Opcodes.RV32_LOAD and funct3 == LoadFunct3.LB
        inst_lh.next     = opcode == Opcodes.RV32_LOAD and funct3 == LoadFunct3.LH
        inst_lw.next     = opcode == Opcodes.RV32_LOAD and funct3 == LoadFunct3.LW
        inst_lbu.next    = opcode == Opcodes.RV32_LOAD and funct3 == LoadFunct3.LBU
        inst_lhu.next    = opcode == Opcodes.RV32_LOAD and funct3 == LoadFunct3.LHU
        #
        inst_sb.next     = opcode == Opcodes.RV32_STORE and funct3 == StoreFunct3.SB
        inst_sh.next     = opcode == Opcodes.RV32_STORE and funct3 == StoreFunct3.SH
        inst_sw.next     = opcode == Opcodes.RV32_STORE and funct3 == StoreFunct3.SW
        #
        inst_addi.next   = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.ADD_SUB
        inst_slti.next   = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.SLT
        inst_sltiu.next  = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.SLTU
        inst_xori.next   = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.XOR
        inst_ori.next    = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.OR
        inst_andi.next   = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.AND
        inst_slli.next   = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.SLL and funct7 == 0
        inst_srli.next   = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.SRL_SRA and funct7 == 0
        inst_srai.next   = opcode == Opcodes.RV32_IMM and funct3 == ArithmeticFunct3.SRL_SRA and funct7 == 0b0100000
        #
        inst_add.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.ADD_SUB and funct7 == 0
        inst_sub.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.ADD_SUB and funct7 == 0b0100000
        inst_sll.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.SLL and funct7 == 0
        inst_slt.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.SLT and funct7 == 0
        inst_sltu.next   = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.SLTU and funct7 == 0
        inst_xor.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.XOR and funct7 == 0
        inst_srl.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.SRL_SRA and funct7 == 0
        inst_sra.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.SRL_SRA and funct7 == 0b0100000
        inst_or.next     = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.OR and funct7 == 0
        inst_and.next    = opcode == Opcodes.RV32_OP and funct3 == ArithmeticFunct3.AND and funct7 == 0
        #
        inst_fence.next  = opcode == Opcodes.RV32_FENCE
        inst_fencei.next = opcode == Opcodes.RV32_FENCE
        #
        inst_csrrw.next  = opcode == Opcodes.RV32_SYSTEM and funct3 == SystemFunct3.CSRRW
        inst_csrrs.next  = opcode == Opcodes.RV32_SYSTEM and funct3 == SystemFunct3.CSRRS
        inst_csrrc.next  = opcode == Opcodes.RV32_SYSTEM and funct3 == SystemFunct3.CSRRC
        inst_csrrwi.next = opcode == Opcodes.RV32_SYSTEM and funct3 == SystemFunct3.CSRRWI
        inst_csrrsi.next = opcode == Opcodes.RV32_SYSTEM and funct3 == SystemFunct3.CSRRSI
        inst_csrrci.next = opcode == Opcodes.RV32_SYSTEM and funct3 == SystemFunct3.CSRRCI
        inst_system.next = opcode == Opcodes.RV32_SYSTEM and funct3 == SystemFunct3.PRIV

    @hdl.always_comb
    def is_xxx_proc():
        is_j.next   = inst_jal or inst_jalr
        is_b.next   = inst_beq or inst_bne or inst_blt or inst_bltu or inst_bge or inst_bgeu
        is_l.next   = inst_lb or inst_lh or inst_lw or inst_lbu or inst_lhu
        is_s.next   = inst_sb or inst_sh or inst_sw
        is_alu.next = inst_addi or inst_slti or inst_sltiu or inst_xori or inst_ori or inst_andi or inst_slli or inst_srli or inst_srai or \
                      inst_add or inst_sub or inst_sll or inst_slt or inst_sltu or inst_xor or inst_srl or inst_sra or inst_or or inst_and  # noqa
        is_csr.next = inst_csrrw or inst_csrrc or inst_csrrs or inst_csrrwi or inst_csrrci or inst_csrrsi

    @hdl.always_comb
    def wb_proc():
        rf_wa.next = rd
        rf_we.next = False
        rf_wd.next = 0
        if state == state_t.WB and not csr_eio.kill_o:
            # can't latch this for now. Need to latch before WB, or make WB a 2 cycle state.
            rf_we.next = is_j or inst_auipc or inst_lui or is_l or is_alu or is_csr
            if is_j:
                rf_wd.next = pc + 4
            elif inst_auipc:
                rf_wd.next = pc + imm_u
            elif inst_lui:
                rf_wd.next = imm_u
            elif is_l:
                rf_wd.next = mdat_i
            elif is_alu:
                rf_wd.next = alu_out
            elif is_csr:
                rf_wd.next = csr_io.rdat_o

    # --------------------------------------------------------------------------
    # branch/jump
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def branch_jump_proc():
        pc_target.next = pc + 4
        if inst_jal:
            pc_target.next = pc + imm_j
        elif inst_jalr:
            pc_target.next = (rs1_d + imm_i) & hdl.modbv(0xFFFFFFFE)[32:]
        elif is_b:
            pc_target.next = pc + imm_b

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def branch_cmp_proc():
        is_eq.next  = rs1_d == rs2_d
        is_lt.next  = rs1_d.signed() < rs2_d.signed()
        is_ltu.next = rs1_d < rs2_d

    @hdl.always_comb
    def take_branch_proc():
        take_branch.next = (is_eq and inst_beq) or (not is_eq and inst_bne) or \
                           (is_lt and inst_blt) or (not is_lt and inst_bge) or \
                           (is_ltu and inst_bltu) or (not is_ltu and inst_bgeu)

    # --------------------------------------------------------------------------
    # ALU
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def alu_assign_proc():
        alu_a.next = rs1_d
        alu_b.next  = imm_i if (inst_addi or inst_slti or inst_sltiu or inst_xori or inst_ori or inst_andi or inst_slli or inst_srli or inst_srai) else rs2_d  # noqa
        if inst_addi or inst_add:
            alu_out.next = alu_a + alu_b
        elif inst_slti or inst_slt:
            alu_out.next = hdl.concat(hdl.modbv(0)[31:], alu_a.signed() < alu_b.signed())
        elif inst_sltiu or inst_sltu:
            alu_out.next = hdl.concat(hdl.modbv(0)[31:], alu_a < alu_b)
        elif inst_xori or inst_xor:
            alu_out.next = alu_a ^ alu_b
        elif inst_ori or inst_or:
            alu_out.next = alu_a | alu_b
        elif inst_andi or inst_and:
            alu_out.next = alu_a & alu_b
        elif inst_slli or inst_sll:
            alu_out.next = alu_a << alu_b[5:]
        elif inst_srli or inst_srl:
            alu_out.next = alu_a >> alu_b[5:]
        elif inst_srai or inst_sra:
            alu_out.next = alu_a.signed() >> alu_b[5:]
        elif inst_sub:
            alu_out.next = alu_a - alu_b

    # --------------------------------------------------------------------------
    # FSM
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def core_fsm_proc():
        if state == state_t.FETCH:
            if exception:
                wb_rw.next = False
                state.next = state_t.WB
            elif wbm.ack_i:
                wb_rw.next       = False
                instruction.next = wbm.dat_i
                state.next       = state_t.DECODE
        elif state == state_t.DECODE:
            enable_csr.next = is_csr or inst_system
            #
            if is_alu:
                state.next = state_t.EX
            elif is_l or is_s:
                if is_s:
                    wb_addr.next = rs1_d + imm_s
                else:
                    wb_addr.next = rs1_d + imm_i
                wbm.dat_o.next = mdat_o
                wb_rw.next     = is_s
                state.next     = state_t.MEM
            else:
                state.next = state_t.WB
        elif state == state_t.EX:
            state.next     = state_t.WB
        elif state == state_t.MEM:
            if wbm.ack_i or exception:
                state.next = state_t.WB
        elif state == state_t.WB:
            pc.next         = pc + 4
            wb_addr.next    = pc + 4
            if csr_eio.kill_o:
                pc.next         = csr_eio.next_pc_o
                wb_addr.next    = csr_eio.next_pc_o
            elif is_j or (is_b and take_branch):
                pc.next         = pc_target
                wb_addr.next    = pc_target
            wbm.dat_o.next    = 0
            wb_rw.next        = False
            instruction.next  = 0x13  # nop
            enable_csr.next   = False
            state.next        = state_t.FETCH
        else:
            wb_addr.next    = rst_addr
            wbm.dat_o.next  = 0
            wb_rw.next      = False
            state.next      = state_t.FETCH

    @hdl.always_comb
    def wb_enable_proc():
        if state == state_t.FETCH or state == state_t.MEM:
            wb_en.next = not wbm.ack_i and not exception
        else:
            wb_en.next = False

    @hdl.always_comb
    def wb_addr_assign_proc():
        wbm.addr_o.next = wb_addr

    # --------------------------------------------------------------------------
    # Memory Read/Write
    @hdl.always_comb
    def wb_write_format_proc():
        mdat_o.next    = (mdat_b if funct3 == StoreFunct3.SB else
                          (mdat_h if funct3 == StoreFunct3.SH else
                           rs2_d))
        wbm.sel_o.next = (0 if is_l else
                          (0x1 << wb_addr[2:0] if funct3 == StoreFunct3.SB else
                           (0x3 << 2 * wb_addr[1] if funct3 == StoreFunct3.SH else
                            0xF)))

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def wb_read_format_proc():
        idxb = wb_addr[2:0]
        idxh = wb_addr[1]
        if funct3 == LoadFunct3.LB:
            mdat_i.next = (wbm.dat_i[8:0].signed() if idxb == 0 else
                           (wbm.dat_i[16:8].signed() if idxb == 1 else
                            (wbm.dat_i[24:16].signed() if idxb == 2 else
                             wbm.dat_i[32:24].signed())))
        elif funct3 == LoadFunct3.LBU:
            mdat_i.next = (wbm.dat_i[8:0] if idxb == 0 else
                           (wbm.dat_i[16:8] if idxb == 1 else
                            (wbm.dat_i[24:16] if idxb == 2 else
                             wbm.dat_i[32:24])))
        elif funct3 == LoadFunct3.LH:
            mdat_i.next = (wbm.dat_i[16:0].signed() if idxh == 0 else
                           wbm.dat_i[32:16].signed())
        elif funct3 == LoadFunct3.LHU:
            mdat_i.next = (wbm.dat_i[16:0] if idxh == 0 else
                           wbm.dat_i[32:16])
        elif funct3 == LoadFunct3.LW:
            mdat_i.next = wbm.dat_i

    # --------------------------------------------------------------------------
    # Exceptions
    @hdl.always_comb
    def exception_flags_proc():
        fetch_misa.next   = pc[2:0] != 0 or ((is_j or take_branch) and pc_target[2:0] != 0)
        fetch_fault.next  = wbm.err_i
        invalid_inst.next = not (is_j or is_b or is_l or is_s or is_alu or is_csr or inst_lui or inst_auipc or inst_system or inst_fencei or inst_fence)
        load_misa.next    = is_l and ((wb_addr[0] and (funct3 == LoadFunct3.LHU or funct3 == LoadFunct3.LH)) or (wb_addr[2:] != 0 and funct3 == LoadFunct3.LW))
        load_fault.next   = wbm.err_i and is_l
        store_misa.next   = is_s and ((wb_addr[0] and funct3 == StoreFunct3.SH) or (wb_addr[2:] != 0 and funct3 == StoreFunct3.SW))
        store_fault.next  = wbm.err_i and is_s

    @hdl.always_comb
    def exception_proc():
        exception.next = invalid_inst or fetch_misa or fetch_fault or load_misa or load_fault or store_misa or store_fault

    # --------------------------------------------------------------------------
    # CSR
    @hdl.always_comb
    def last_cycle_proc():
        last_cycle.next = state == state_t.WB

    @hdl.always_comb
    def csr_io_proc():
        csr_io.addr_i.next   = imm12
        csr_io.wdat_i.next   = rs1_d if (inst_csrrw or inst_csrrc or inst_csrrs) else rs1
        csr_io.r_i.next      = (inst_csrrc or inst_csrrs or inst_csrrci or inst_csrrsi) and rs1 == 0  # this is useless
        csr_io.w_i.next      = inst_csrrw or inst_csrrwi
        csr_io.s_i.next      = (inst_csrrs or inst_csrrsi) and rs1 != 0
        csr_io.c_i.next      = (inst_csrrc or inst_csrrci) and rs1 != 0
        csr_io.system_i.next = inst_system
        #
        csr_eio.exception_i.next = exception
        csr_eio.exception_dat_i.next  = pc_target if is_j or take_branch else wb_addr
        if fetch_misa:
            csr_eio.exception_code_i.next = ExceptionCode.E_INST_ADDR_MISALIGNED
        elif fetch_fault:
            csr_eio.exception_code_i.next = ExceptionCode.E_INST_ACCESS_FAULT
        elif load_misa:
            csr_eio.exception_code_i.next = ExceptionCode.E_LOAD_ADDR_MISALIGNED
        elif load_fault:
            csr_eio.exception_code_i.next = ExceptionCode.E_LOAD_ACCESS_FAULT
        elif store_misa:
            csr_eio.exception_code_i.next = ExceptionCode.E_STORE_AMO_ADDR_MISALIGNED
        elif store_fault:
            csr_eio.exception_code_i.next = ExceptionCode.E_STORE_AMO_ACCESS_FAULT
        else:
            csr_eio.exception_code_i.next = ExceptionCode.E_ILLEGAL_INST
            csr_eio.exception_dat_i.next  = instruction
        csr_eio.exception_pc_i.next   = pc

    return hdl.instances()


if __name__ == '__main__':
    config = Configuration('tests/core/algol_RV32I.ini')
    clk    = createSignal(0, 1)
    rst    = hdl.ResetSignal(0, active=True, async=False)
    wbp    = WishboneIntercon()
    ci     = CoreInterrupts()
    core   = CoreB(clk, rst, wbp, ci, None, 0, config)
    core.convert(testbench=False)

# Local Variables:
# flycheck-flake8-maximum-line-length: 300
# flycheck-flake8rc: ".flake8rc"
# End:
