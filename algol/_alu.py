#!/usr/bin/env python3
# Copyright (c) 2017 Angel Terrones <angelterrones@gmail.com>

import myhdl as hdl


class ALUOp:
    """ALU opcodes"""
    SZ_OP = 4
    ADD   = 0
    SLL   = 1
    XOR   = 2
    SRL   = 3
    OR    = 4
    AND   = 5
    SUB   = 6
    SRA   = 7
    SLT   = 8
    SLTU  = 9


@hdl.block
def ALU(a_i, b_i, op_i, res_o):
    @hdl.always_comb
    def rtl_proc():
        if op_i == ALUOp.ADD:
            res_o.next = a_i + b_i
        elif op_i == ALUOp.SLL:
            res_o.next = a_i << b_i[5:0]
        elif op_i == ALUOp.XOR:
            res_o.next = a_i ^ b_i
        elif op_i == ALUOp.SRL:
            res_o.next = a_i >> b_i[5:0]
        elif op_i == ALUOp.OR:
            res_o.next = a_i | b_i
        elif op_i == ALUOp.AND:
            res_o.next = a_i & b_i
        elif op_i == ALUOp.SUB:
            res_o.next = a_i - b_i
        elif op_i == ALUOp.SRA:
            res_o.next = a_i.signed() >> b_i[5:0]
        elif op_i == ALUOp.SLT:
            res_o.next = hdl.concat(hdl.modbv(0)[31:], a_i.signed() < b_i.signed())
        elif op_i == ALUOp.SLTU:
            res_o.next = hdl.concat(hdl.modbv(0)[31:], a_i < b_i)
        else:
            res_o.next = 0

    return hdl.instances()

# Local Variables:
# flycheck-flake8-maximum-line-length: 200
# flycheck-flake8rc: ".flake8rc"
# End:
