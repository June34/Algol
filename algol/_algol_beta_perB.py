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

    rst_addr      = config.getOption('Core', 'start_address')
    # FSM
    state_t       = hdl.enum('RST', 'FETCH', 'DECODE', 'EX', 'LD', 'ST', 'EXC', 'WB')
    state         = hdl.Signal(state_t.RST)
    # WB signals
    wb_addr       = createSignal(0, 32)
    wbm           = WishboneMaster(wb_port)
    # Fetch
    pc            = createSignal(rst_addr, 32)
    pcp4          = createSignal(rst_addr, 32)
    instruction   = createSignal(0, 32)
    instruction_r = createSignal(0x13, 32)  # default value = nop instruction
    #
    opcode_q      = instruction(7, 0)
    funct3_q      = instruction(15, 12)
    funct7_q      = instruction(32, 25)
    rs1_q         = instruction(20, 15)
    rs2_q         = instruction(25, 20)
    rs1           = instruction_r(20, 15)
    rs2           = instruction_r(25, 20)
    rd            = instruction_r(12, 7)
    imm12         = instruction_r(32, 20)
    funct3        = instruction_r(15, 12)
    imm_i         = hdl.ConcatSignal(*[instruction_r(31) for _ in range(21)], instruction_r(31, 20))
    imm_s         = hdl.ConcatSignal(*[instruction_r(31) for _ in range(21)], instruction_r(31, 25), instruction_r(12, 7))
    imm_b         = hdl.ConcatSignal(*[instruction_r(31) for _ in range(20)], instruction_r(7), instruction_r(31, 25), instruction_r(12, 8), False)
    imm_u         = hdl.ConcatSignal(instruction_r(32, 12), hdl.modbv(0, _nrbits=12))
    imm_j         = hdl.ConcatSignal(*(instruction_r(31) for _ in range(12)), instruction_r(20, 12), instruction_r(20), instruction_r(31, 21), False)
    # decoder
    inst_lui      = createSignal(0, 1)
    inst_auipc    = createSignal(0, 1)
    inst_jal      = createSignal(0, 1)
    inst_jalr     = createSignal(0, 1)
    inst_beq      = createSignal(0, 1)
    inst_bne      = createSignal(0, 1)
    inst_blt      = createSignal(0, 1)
    inst_bge      = createSignal(0, 1)
    inst_bltu     = createSignal(0, 1)
    inst_bgeu     = createSignal(0, 1)
    inst_lb       = createSignal(0, 1)
    inst_lh       = createSignal(0, 1)
    inst_lw       = createSignal(0, 1)
    inst_lbu      = createSignal(0, 1)
    inst_lhu      = createSignal(0, 1)
    inst_sb       = createSignal(0, 1)
    inst_sh       = createSignal(0, 1)
    inst_sw       = createSignal(0, 1)
    inst_addi     = createSignal(0, 1)
    inst_slti     = createSignal(0, 1)
    inst_sltiu    = createSignal(0, 1)
    inst_xori     = createSignal(0, 1)
    inst_ori      = createSignal(0, 1)
    inst_andi     = createSignal(0, 1)
    inst_slli     = createSignal(0, 1)
    inst_srli     = createSignal(0, 1)
    inst_srai     = createSignal(0, 1)
    inst_add      = createSignal(0, 1)
    inst_sub      = createSignal(0, 1)
    inst_sll      = createSignal(0, 1)
    inst_slt      = createSignal(0, 1)
    inst_sltu     = createSignal(0, 1)
    inst_xor      = createSignal(0, 1)
    inst_srl      = createSignal(0, 1)
    inst_sra      = createSignal(0, 1)
    inst_or       = createSignal(0, 1)
    inst_and      = createSignal(0, 1)
    inst_fence    = createSignal(0, 1)
    inst_fencei   = createSignal(0, 1)
    inst_csrrw    = createSignal(0, 1)
    inst_csrrs    = createSignal(0, 1)
    inst_csrrc    = createSignal(0, 1)
    inst_csrrwi   = createSignal(0, 1)
    inst_csrrsi   = createSignal(0, 1)
    inst_csrrci   = createSignal(0, 1)
    inst_system   = createSignal(0, 1)
    is_j          = createSignal(0, 1)
    is_b          = createSignal(0, 1)
    is_l          = createSignal(0, 1)
    is_s          = createSignal(0, 1)
    is_alu        = createSignal(0, 1)
    is_csr        = createSignal(0, 1)
    #
    is_eq         = createSignal(0, 1)
    is_lt         = createSignal(0, 1)
    is_ltu        = createSignal(0, 1)
    take_branch   = createSignal(0, 1)
    fetch_misa    = createSignal(0, 1)
    fetch_fault   = createSignal(0, 1)
    invalid_inst  = createSignal(0, 1)
    load_misa     = createSignal(0, 1)
    load_fault    = createSignal(0, 1)
    store_misa    = createSignal(0, 1)
    store_fault   = createSignal(0, 1)
    exception     = createSignal(0, 1)
    # Register file
    rs1_d         = createSignal(0, 32)
    rs2_d         = createSignal(0, 32)
    rf_we         = createSignal(0, 1)
    rf_wd         = createSignal(0, 32)
    regfile       = [createSignal(0, 32) for _ in range(32)]
    # Branch/jump
    pc_target     = createSignal(0, 32)
    # CSR
    csr_retire    = createSignal(0, 1)
    enable_csr    = createSignal(0, 1)
    csr_ack       = createSignal(0, 1)
    csr_io        = CSRIO()
    csr_eio       = CSRExceptionIO()
    csr           = CSR(clk_i=clk_i, rst_i=rst_i, retire_i=csr_retire, enable_i=enable_csr, ack_o=csr_ack,  # noqa
                        io=csr_io, eio=csr_eio, core_interrupts=core_interrupts, hart_id=hart_id, config=config)
    # ALU
    alu_a         = createSignal(0, 32)
    alu_b         = createSignal(0, 32)
    alu_out       = createSignal(0, 32)
    alu_add_sub   = createSignal(0, 32)
    alu_shift     = createSignal(0, 32)
    alu_logic     = createSignal(0, 32)
    alu_cmp       = createSignal(0, 1)
    is_add_sub    = createSignal(0, 1)
    is_shift      = createSignal(0, 1)
    is_logic      = createSignal(0, 1)
    is_cmp        = createSignal(0, 1)
    shamt         = createSignal(0, 5)
    shift_done    = createSignal(0, 1)
    shift_busy    = createSignal(0, 1)
    shitf_o       = createSignal(0, 32)
    # memory
    mem_sel       = createSignal(0, 4)
    is_mb         = createSignal(0, 1)
    is_mbu        = createSignal(0, 1)
    is_mh         = createSignal(0, 1)
    is_mhu        = createSignal(0, 1)
    is_mw         = createSignal(0, 1)
    mdat_b        = hdl.ConcatSignal(rs2_d(8, 0), rs2_d(8, 0), rs2_d(8, 0), rs2_d(8, 0))
    mdat_h        = hdl.ConcatSignal(rs2_d(16, 0), rs2_d(16, 0))
    mdat_o        = createSignal(0, 32)
    mdat_i        = createSignal(0, 32)

    # --------------------------------------------------------------------------
    # decoder
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def decoder_proc():
        if state == state_t.FETCH:
            inst_lui.next    = opcode_q == Opcodes.RV32_LUI
            inst_auipc.next  = opcode_q == Opcodes.RV32_AUIPC
            #
            inst_jal.next    = opcode_q == Opcodes.RV32_JAL
            inst_jalr.next   = opcode_q == Opcodes.RV32_JALR
            #
            inst_beq.next    = opcode_q == Opcodes.RV32_BRANCH and funct3_q == BranchFunct3.BEQ
            inst_bne.next    = opcode_q == Opcodes.RV32_BRANCH and funct3_q == BranchFunct3.BNE
            inst_blt.next    = opcode_q == Opcodes.RV32_BRANCH and funct3_q == BranchFunct3.BLT
            inst_bge.next    = opcode_q == Opcodes.RV32_BRANCH and funct3_q == BranchFunct3.BGE
            inst_bltu.next   = opcode_q == Opcodes.RV32_BRANCH and funct3_q == BranchFunct3.BLTU
            inst_bgeu.next   = opcode_q == Opcodes.RV32_BRANCH and funct3_q == BranchFunct3.BGEU
            #
            inst_lb.next     = opcode_q == Opcodes.RV32_LOAD and funct3_q == LoadFunct3.LB
            inst_lh.next     = opcode_q == Opcodes.RV32_LOAD and funct3_q == LoadFunct3.LH
            inst_lw.next     = opcode_q == Opcodes.RV32_LOAD and funct3_q == LoadFunct3.LW
            inst_lbu.next    = opcode_q == Opcodes.RV32_LOAD and funct3_q == LoadFunct3.LBU
            inst_lhu.next    = opcode_q == Opcodes.RV32_LOAD and funct3_q == LoadFunct3.LHU
            #
            inst_sb.next     = opcode_q == Opcodes.RV32_STORE and funct3_q == StoreFunct3.SB
            inst_sh.next     = opcode_q == Opcodes.RV32_STORE and funct3_q == StoreFunct3.SH
            inst_sw.next     = opcode_q == Opcodes.RV32_STORE and funct3_q == StoreFunct3.SW
            #
            inst_addi.next   = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.ADD_SUB
            inst_slti.next   = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.SLT
            inst_sltiu.next  = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.SLTU
            inst_xori.next   = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.XOR
            inst_ori.next    = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.OR
            inst_andi.next   = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.AND
            inst_slli.next   = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.SLL and funct7_q == 0
            inst_srli.next   = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.SRL_SRA and funct7_q == 0
            inst_srai.next   = opcode_q == Opcodes.RV32_IMM and funct3_q == ArithmeticFunct3.SRL_SRA and funct7_q == 0b0100000
            #
            inst_add.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.ADD_SUB and funct7_q == 0
            inst_sub.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.ADD_SUB and funct7_q == 0b0100000
            inst_sll.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.SLL and funct7_q == 0
            inst_slt.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.SLT and funct7_q == 0
            inst_sltu.next   = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.SLTU and funct7_q == 0
            inst_xor.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.XOR and funct7_q == 0
            inst_srl.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.SRL_SRA and funct7_q == 0
            inst_sra.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.SRL_SRA and funct7_q == 0b0100000
            inst_or.next     = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.OR and funct7_q == 0
            inst_and.next    = opcode_q == Opcodes.RV32_OP and funct3_q == ArithmeticFunct3.AND and funct7_q == 0
            #
            inst_fence.next  = opcode_q == Opcodes.RV32_FENCE
            inst_fencei.next = opcode_q == Opcodes.RV32_FENCE
            #
            inst_csrrw.next  = opcode_q == Opcodes.RV32_SYSTEM and funct3_q == SystemFunct3.CSRRW
            inst_csrrs.next  = opcode_q == Opcodes.RV32_SYSTEM and funct3_q == SystemFunct3.CSRRS
            inst_csrrc.next  = opcode_q == Opcodes.RV32_SYSTEM and funct3_q == SystemFunct3.CSRRC
            inst_csrrwi.next = opcode_q == Opcodes.RV32_SYSTEM and funct3_q == SystemFunct3.CSRRWI
            inst_csrrsi.next = opcode_q == Opcodes.RV32_SYSTEM and funct3_q == SystemFunct3.CSRRSI
            inst_csrrci.next = opcode_q == Opcodes.RV32_SYSTEM and funct3_q == SystemFunct3.CSRRCI
            inst_system.next = opcode_q == Opcodes.RV32_SYSTEM and funct3_q == SystemFunct3.PRIV

    @hdl.always_comb
    def instruction_assign_proc():
        instruction.next = wbm.dat_i

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def is_xxx_proc():
        is_j.next   = inst_jal or inst_jalr
        is_b.next   = inst_beq or inst_bne or inst_blt or inst_bltu or inst_bge or inst_bgeu
        is_l.next   = inst_lb or inst_lh or inst_lw or inst_lbu or inst_lhu
        is_s.next   = inst_sb or inst_sh or inst_sw
        is_alu.next = inst_addi or inst_slti or inst_sltiu or inst_xori or inst_ori or inst_andi or inst_slli or inst_srli or inst_srai or \
                      inst_add or inst_sub or inst_sll or inst_slt or inst_sltu or inst_xor or inst_srl or inst_sra or inst_or or inst_and  # noqa
        is_csr.next = inst_csrrw or inst_csrrc or inst_csrrs or inst_csrrwi or inst_csrrci or inst_csrrsi

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def is_alu_xxx_proc():
        is_add_sub.next = inst_add or inst_addi or inst_sub
        is_shift.next   = inst_slli or inst_sll or inst_srli or inst_srl or inst_srai or inst_sra
        is_logic.next   = inst_xori or inst_xor or inst_ori or inst_or or inst_andi or inst_and
        is_cmp.next     = inst_slti or inst_slt or inst_sltiu or inst_sltu

    # Add delays @ DECODE/EXECUTE/WRITEBACK
    dec_state = createSignal(0, 1)
    ex_state  = createSignal(0, 1)
    wb_state  = createSignal(0, 1)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def core_fsm_proc():
        if state == state_t.RST:
            ex_state.next  = False
            wbm.cyc_o.next = True
            wbm.stb_o.next = True
            state.next     = state_t.FETCH
        elif state == state_t.FETCH:
            # fetch_misa.next  = False
            # fetch_fault.next = False
            if pc[2:0] != 0 or wbm.err_i:
                fetch_misa.next  = pc[2:] != 0
                fetch_fault.next = wbm.err_i
                wbm.cyc_o.next   = False
                wbm.stb_o.next   = False
                state.next       = state_t.EXC
            elif wbm.ack_i:
                wbm.cyc_o.next     = False
                wbm.stb_o.next     = False
                instruction_r.next = instruction
                state.next         = state_t.DECODE
        elif state == state_t.DECODE:
            dec_state.next = True
            if dec_state:
                dec_state.next = False
                if invalid_inst:
                    state.next = state_t.EXC
                elif is_csr or inst_system:
                    enable_csr.next = True
                    state.next = state_t.WB
                elif is_l:
                    state.next = state_t.LD
                elif is_s:
                    state.next = state_t.ST
                else:
                    state.next = state_t.EX  # calculate target
        elif state == state_t.EX:
            if is_shift:
                if shift_done:
                    state.next = state_t.WB
            elif is_j:
                if pc_target[2:0] != 0:
                    fetch_misa.next = True
                    state.next = state_t.EXC
                else:
                    fetch_misa.next = False
                    state.next = state_t.WB
            else:
                ex_state.next = True
                if ex_state:
                    ex_state.next = False
                    state.next = state_t.WB
                    if take_branch:
                        if pc_target[2:] != 0:
                            fetch_misa.next = True
                            state.next = state_t.EXC
        elif state == state_t.LD:
            wbm.cyc_o.next  = True
            wbm.stb_o.next  = True
            misa = (wb_addr[0] and (is_mh or is_mhu) or (wb_addr[2:] != 0 and is_mw))
            if misa or wbm.err_i:
                wbm.cyc_o.next  = False
                wbm.stb_o.next  = False
                enable_csr.next = True
                load_misa.next  = misa
                load_fault.next = wbm.err_i
                state.next      = state_t.EXC
            elif wbm.ack_i:
                wbm.cyc_o.next  = False
                wbm.stb_o.next  = False
                state.next = state_t.WB
        elif state == state_t.ST:
            wbm.we_o.next   = True
            wbm.cyc_o.next  = True
            wbm.stb_o.next  = True
            misa = (wb_addr[0] and (is_mh or is_mhu) or (wb_addr[2:] != 0 and is_mw))
            if misa or wbm.err_i:
                wbm.we_o.next    = False
                wbm.cyc_o.next   = False
                wbm.stb_o.next   = False
                store_misa.next  = misa
                store_fault.next = wbm.err_i
                enable_csr.next  = True
                state.next       = state_t.EXC
            elif wbm.ack_i:
                wbm.we_o.next   = False
                wbm.cyc_o.next  = False
                wbm.stb_o.next  = False
                state.next = state_t.WB
        elif state == state_t.EXC:
            enable_csr.next = True
            state.next      = state_t.WB
        elif state == state_t.WB:
            if is_csr or inst_system or exception:
                if csr_eio.kill_o:
                    enable_csr.next  = False
                    fetch_misa.next  = False
                    fetch_fault.next = False
                    load_misa.next   = False
                    load_fault.next  = False
                    store_misa.next  = False
                    store_fault.next = False
                    pc.next          = csr_eio.next_pc_o
                    wbm.cyc_o.next   = True
                    wbm.stb_o.next   = True
                    state.next       = state_t.FETCH
                elif csr_ack:
                    enable_csr.next = False
                    pc.next         = pc_target
                    wbm.cyc_o.next  = True
                    wbm.stb_o.next  = True
                    state.next      = state_t.FETCH
            else:
                wb_state.next   = True
                csr_retire.next = True
                if wb_state:
                    csr_retire.next = False
                    enable_csr.next = False
                    pc.next         = pc_target
                    wb_state.next   = False
                    wbm.cyc_o.next  = True
                    wbm.stb_o.next  = True
                    state.next      = state_t.FETCH
        else:
            enable_csr.next = False
            state.next = state_t.FETCH

    # --------------------------------------------------------------------------
    # register file
    _rs1 = createSignal(0, 5)
    _rs2 = createSignal(0, 5)
    _rwa = createSignal(0, 5)
    _rwa._markRead()
    inst_writes = createSignal(0, 1)
    wb_data     = createSignal(0, 32)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def rf_read_proc():
        rs1_d.next = regfile[_rs1] if _rs1 != 0 else 0
        rs2_d.next = regfile[_rs2] if _rs2 != 0 else 0

    @hdl.always(clk_i.posedge)
    def rf_write_proc():
        if rf_we and rd != 0:
            regfile[rd].next = rf_wd

    @hdl.always_comb
    def fuck_proc():
        _rs1.next = rs1_q if state == state_t.FETCH else rs1
        _rs2.next = rs2_q if state == state_t.FETCH else rs2
        _rwa.next = rd

    @hdl.always(clk_i.posedge)
    def inst_writes_rf_proc():
        inst_writes.next = not csr_eio.kill_o and (is_j or inst_auipc or inst_lui or is_l or is_alu or is_csr)
        if is_j:
            wb_data.next = pcp4
        elif inst_auipc:
            wb_data.next = pc + imm_u
        elif inst_lui:
            wb_data.next = imm_u
        elif is_l:
            wb_data.next = mdat_i
        elif is_csr:
            wb_data.next = csr_io.rdat_o
        else:
            wb_data.next = alu_out

    @hdl.always_comb
    def rf_set_write_data_proc():
        rf_wd.next = wb_data
        rf_we.next = inst_writes and state == state_t.WB and (wb_state or csr_ack)

    @hdl.always(clk_i.posedge)
    def pc_plus_4_pr2oc():
        pcp4.next = pc + 4

    # --------------------------------------------------------------------------
    # branch/jump
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def branch_cmp_proc():
        is_eq.next  = rs1_d == rs2_d
        is_lt.next  = rs1_d.signed() < rs2_d.signed()
        is_ltu.next = rs1_d < rs2_d

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def take_branch_proc():
        take_branch.next = (is_eq and inst_beq) or (not is_eq and inst_bne) or \
                           (is_lt and inst_blt) or (not is_lt and inst_bge) or \
                           (is_ltu and inst_bltu) or (not is_ltu and inst_bgeu)

    pc_j  = createSignal(0, 32)
    pc_jr = createSignal(0, 32)
    pc_b  = createSignal(0, 32)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def pc_target_calc_proc():
        pc_j.next  = pc + imm_j
        pc_jr.next = (rs1_d + imm_i) & hdl.modbv(0xFFFFFFFE)[32:]
        pc_b.next  = pc + imm_b
        #
        pc_target.next = pcp4  # TODO: check
        if inst_jal:
            pc_target.next = pc_j
        elif inst_jalr:
            pc_target.next = pc_jr
        elif take_branch:
            pc_target.next = pc_b

    # --------------------------------------------------------------------------
    # ALU
    # Add/sub, logic, shifter and compare
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def latch_alu_a_proc():
        alu_a.next = rs1_d

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def latch_alu_b_proc():
        if (inst_addi or inst_slti or inst_sltiu or inst_xori or inst_ori or inst_andi or inst_slli or inst_srli or inst_srai):
            alu_b.next = imm_i
        else:
            alu_b.next = rs2_d

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def alu_op_proc():
        if inst_sub:
            alu_add_sub.next = alu_a - alu_b
        else:
            alu_add_sub.next = alu_a + alu_b

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def alu_shift_proc():
        if not shift_busy:
            shift_done.next = False
            shitf_o.next    = alu_a
            shamt.next      = alu_b[5:0]
            shift_busy.next = is_shift
        elif shamt >= 4:
            if inst_slli or inst_sll:
                shitf_o.next = shitf_o << 4
            elif inst_srli or inst_srl:
                shitf_o.next = shitf_o >> 4
            elif inst_srai or inst_sra:
                shitf_o.next = shitf_o.signed() >> 4
            shamt.next = shamt - 4
        elif shamt > 0:
            if inst_slli or inst_sll:
                shitf_o.next = shitf_o << 1
            elif inst_srli or inst_srl:
                shitf_o.next = shitf_o >> 1
            elif inst_srai or inst_sra:
                shitf_o.next = shitf_o.signed() >> 1
            shamt.next = shamt - 1
        else:
            alu_shift.next  = shitf_o
            shift_busy.next = False
            shift_done.next = True

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def alu_logic_proc():
        if inst_xori or inst_xor:
            alu_logic.next = alu_a ^ alu_b
        elif inst_ori or inst_or:
            alu_logic.next = alu_a | alu_b
        elif inst_andi or inst_and:
            alu_logic.next = alu_a & alu_b

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def alu_cmp_proc():
        if inst_slti or inst_slt:
            alu_cmp.next = alu_a.signed() < alu_b.signed()
        elif inst_sltiu or inst_sltu:
            alu_cmp.next = alu_a < alu_b

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def alu_proc():
        if is_add_sub:
            alu_out.next = alu_add_sub
        elif is_logic:
            alu_out.next = alu_logic
        elif is_cmp:
            alu_out.next = alu_cmp
        else:
            alu_out.next = alu_shift

    # --------------------------------------------------------------------------
    # Memory Read/Write
    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def check_mem_operation_proc():
        is_mb.next  = funct3 == LoadFunct3.LB or funct3 == StoreFunct3.SB
        is_mbu.next = funct3 == LoadFunct3.LBU
        is_mh.next  = funct3 == LoadFunct3.LH or funct3 == StoreFunct3.SH
        is_mhu.next = funct3 == LoadFunct3.LHU
        is_mw.next  = funct3 == LoadFunct3.LW or funct3 == StoreFunct3.SW

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def mem_addr_proc():
        if state == state_t.DECODE:
            if is_l:
                wb_addr.next = rs1_d + imm_i
            else:
                wb_addr.next = rs1_d + imm_s

    @hdl.always_comb
    def wb_write_format_proc():
        mdat_o.next = (mdat_b if is_mb else
                       (mdat_h if is_mh else
                        rs2_d))
        mem_sel.next = (0 if is_l else
                        (0x1 << wb_addr[2:0] if is_mb else
                         (0x3 << 2 * wb_addr[1] if is_mh else
                          0xF)))

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def wb_read_format_proc():
        idxb = wb_addr[2:0]
        idxh = wb_addr[1]
        if is_mb:
            if idxb == 0:
                mdat_i.next = wbm.dat_i[8:0].signed()
            elif idxb == 1:
                mdat_i.next = wbm.dat_i[16:8].signed()
            elif idxb == 2:
                mdat_i.next = wbm.dat_i[24:16].signed()
            else:
                mdat_i.next = wbm.dat_i[32:24].signed()
        elif is_mbu:
            if idxb == 0:
                mdat_i.next = wbm.dat_i[8:0]
            elif idxb == 1:
                mdat_i.next = wbm.dat_i[16:8]
            elif idxb == 2:
                mdat_i.next = wbm.dat_i[24:16]
            else:
                mdat_i.next = wbm.dat_i[32:24]
        elif is_mh:
            if idxh == 0:
                mdat_i.next = wbm.dat_i[16:0].signed()
            else:
                mdat_i.next = wbm.dat_i[32:16].signed()
        elif is_mhu:
            if idxh == 0:
                mdat_i.next = wbm.dat_i[16:0]
            else:
                mdat_i.next = wbm.dat_i[32:16]
        elif funct3 == LoadFunct3.LW:
            mdat_i.next = wbm.dat_i

    # --------------------------------------------------------------------------
    # Wishbone port handling
    @hdl.always_comb
    def wb_port_assign_proc():
        wbm.addr_o.next = 0
        wbm.dat_o.next  = 0
        wbm.sel_o.next  = mem_sel
        if state == state_t.FETCH:
            wbm.addr_o.next = pc
        elif state == state_t.LD:
            wbm.addr_o.next = wb_addr
        elif state == state_t.ST:
            wbm.addr_o.next = wb_addr
            wbm.dat_o.next  = mdat_o

    # --------------------------------------------------------------------------
    # Exceptions
    @hdl.always_comb
    def exception_flags_proc():
        invalid_inst.next = not (is_j or is_b or is_l or is_s or is_alu or is_csr or inst_lui or inst_auipc or inst_system or inst_fencei or inst_fence)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def exception_proc():
        exception.next = False
        if state == state_t.EXC or state == state_t.WB:
            exception.next = invalid_inst or fetch_misa or fetch_fault or load_misa or load_fault or store_misa or store_fault

    is_csr_sc = createSignal(0, 1)
    is_csrs   = createSignal(0, 1)
    is_csrc   = createSignal(0, 1)
    is_csrx   = createSignal(0, 1)

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def check_csr_instruction_proc():
        is_csr_sc.next = inst_csrrc or inst_csrrs or inst_csrrci or inst_csrrsi
        is_csrs.next   = inst_csrrs or inst_csrrsi
        is_csrc.next   = inst_csrrc or inst_csrrci
        is_csrx.next   = inst_csrrw or inst_csrrc or inst_csrrs

    @hdl.always_seq(clk_i.posedge, reset=rst_i)
    def csr_io_proc():
        csr_io.addr0_i.next  = imm12
        csr_io.addr1_i.next  = imm12
        csr_io.addr2_i.next  = imm12
        csr_io.wdat_i.next   = rs1_d if is_csrx else rs1
        csr_io.r_i.next      = is_csr_sc and rs1 == 0  # this is useless
        csr_io.w_i.next      = inst_csrrw or inst_csrrwi
        csr_io.s_i.next      = is_csrs and rs1 != 0
        csr_io.c_i.next      = is_csrc and rs1 != 0
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
            csr_eio.exception_dat_i.next  = instruction_r
        csr_eio.exception_pc_i.next   = pc

    return hdl.instances()


@hdl.block
def CoreB_HDL(clk_i, rst_i,
              wbm_addr_o, wbm_dat_o, wbm_dat_i, wbm_sel_o, wbm_cyc_o, wbm_we_o, wbm_stb_o, wbm_ack_i, wbm_err_i,
              meip_i, mtip_i, msip_i, hart_id, config):
    """Algol Core B

    For sysnthesis and cosimulation.
    """
    assert isinstance(config, Configuration), "[Algol Core B] Error: config data mustbe of type Configuration"
    assert hart_id >= 0, "[Algol Core B] Error: HART_ID must be >= 0"

    wbp    = WishboneIntercon()
    ci     = CoreInterrupts()
    core   = CoreB(clk_i=clk_i, rst_i=rst_i, wb_port=wbp, core_interrupts=ci, debug=None, hart_id=hart_id, config=config)  # noqa

    @hdl.always_comb
    def port_assign_proc():
        wbm_addr_o.next = wbp.addr
        wbm_dat_o.next  = wbp.dat_o
        wbm_sel_o.next  = wbp.sel
        wbm_cyc_o.next  = wbp.cyc
        wbm_we_o.next   = wbp.we
        wbm_stb_o.next  = wbp.stb
        wbp.dat_i.next  = wbm_dat_i
        wbp.ack.next    = wbm_ack_i
        wbp.err.next    = wbm_err_i
        ci.meip.next    = meip_i
        ci.mtip.next    = mtip_i
        ci.msip.next    = msip_i

    return hdl.instances()


def generate_verilog(name, path, config_file, trace, testbench):
    config = Configuration(config_file)
    clk             = createSignal(0, 1)
    rst             = hdl.ResetSignal(0, active=True, async=False)
    wb_port         = WishboneIntercon()
    core_interrupts = CoreInterrupts()
    core = CoreB_HDL(clk_i=clk,
                     rst_i=rst,
                     wbm_addr_o=wb_port.addr,
                     wbm_dat_o=wb_port.dat_o,
                     wbm_dat_i=wb_port.dat_i,
                     wbm_sel_o=wb_port.sel,
                     wbm_cyc_o=wb_port.cyc,
                     wbm_we_o=wb_port.we,
                     wbm_stb_o=wb_port.stb,
                     wbm_ack_i=wb_port.ack,
                     wbm_err_i=wb_port.err,
                     meip_i=core_interrupts.meip,
                     mtip_i=core_interrupts.mtip,
                     msip_i=core_interrupts.msip,
                     hart_id=0,
                     config=config)
    core.convert(path=path, name=name, trace=trace, testbench=testbench)


if __name__ == '__main__':
    generate_verilog('CoreB', '.', 'tests/core/algol_RV32I.ini', False, False)

# Local Variables:
# flycheck-flake8-maximum-line-length: 300
# flycheck-flake8rc: ".flake8rc"
# End:
