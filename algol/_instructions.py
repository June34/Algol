#!/usr/bin/env python
# Copyright (c) 2017 Angel Terrones <angelterrones@gmail.com>


class Opcodes:
    # RV32I
    RV32_NOP      = 0b0010011
    RV32_LUI      = 0b0110111
    RV32_AUIPC    = 0b0010111
    RV32_JAL      = 0b1101111
    RV32_JALR     = 0b1100111
    RV32_BRANCH   = 0b1100011
    RV32_LOAD     = 0b0000011
    RV32_STORE    = 0b0100011
    RV32_IMM      = 0b0010011
    RV32_OP       = 0b0110011  # same for RV32M
    RV32_FENCE    = 0b0001111
    RV32_SYSTEM   = 0b1110011
    # RV32A
    RV32_AMO      = 0b0101111
    # custom
    RV32_CUSTOM_0 = 0b0001011
    RV32_CUSTOM_1 = 0b0101011


class BranchFunct3:
    BEQ  = 0
    BNE  = 1
    BLT  = 4
    BGE  = 5
    BLTU = 6
    BGEU = 7


class LoadFunct3:
    LB  = 0
    LH  = 1
    LW  = 2
    LBU = 4
    LHU = 5


class StoreFunct3:
    SB = 0
    SH = 1
    SW = 2


class ArithmeticFunct3:
    ADD_SUB = 0
    SLL     = 1
    SLT     = 2
    SLTU    = 3
    XOR     = 4
    SRL_SRA = 5
    OR      = 6
    AND     = 7


class FenceFunct3:
    FENCE   = 0
    FENCE_I = 1


class SystemFunct3:
    PRIV   = 0
    CSRRW  = 1
    CSRRS  = 2
    CSRRC  = 3
    CSRRWI = 5
    CSRRSI = 6
    CSRRCI = 7


class PrivFunct12:
    ECALL  = 0b000000000000
    EBREAK = 0b000000000001
    ERET   = 0b000100000000


class MulDivFunct:
    MUL_DIV = 0b0000001
    MUL     = 0
    MULH    = 1
    MULHSU  = 2
    MULHU   = 3
    DIV     = 4
    DIVU    = 5
    REM     = 6
    REMU    = 7


class AMOFunct:
    AMO       = 0b010
    LR_W      = 0b00010
    SC_W      = 0b00011
    AMOSWAP_W = 0b00001
    AMOADD_W  = 0b00000
    AMOXOR_W  = 0b00100
    AMOAND_W  = 0b01100
    AMOOR_W   = 0b01000
    AMOMIN_W  = 0b10000
    AMOMAX_W  = 0b10100
    AMOMINU_W = 0b11000
    AMOMAXU_W = 0b11100


# Local Variables:
# flycheck-flake8-maximum-line-length: 200
# flycheck-flake8rc: ".flake8rc"
# End:
