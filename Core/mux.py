#!/usr/bin/env python
# Copyright (c) 2015 Angel Terrones (<angelterrones@gmail.com>)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from myhdl import Signal
from myhdl import always_comb


class Mux:
    def __init__(self,
                 sel:    Signal,
                 inputs: [Signal],
                 out:    Signal):
        # Check valid settings
        assert len(inputs) >= 2, "Inputs must be >= 2"

        self.sel = sel
        self.inputs = inputs
        self.out = out

    def GetRTL(self):
        @always_comb
        def rtl():
            assert self.sel <= len(self.inputs), "Select is out of bounds."
            self.out.next = self.inputs[self.sel]

        return rtl


class Mux2:
    def __init__(self,
                 sel: Signal,
                 in1: Signal,
                 in2: Signal,
                 out: Signal):
        self.sel = sel
        self.in1 = in1
        self.in2 = in2
        self.out = out

    def GetRTL(self):
        @always_comb
        def rtl():
            if self.sel == 0:
                self.out.next = self.in1
            else:
                self.out.next = self.in2

        return rtl


class Mux4:
    def __init__(self,
                 sel: Signal,
                 in1: Signal,
                 in2: Signal,
                 in3: Signal,
                 in4: Signal,
                 out: Signal):
        self.sel = sel
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.out = out

    def GetRTL(self):
        @always_comb
        def rtl():
            if self.sel == 0:
                self.out.next = self.in1
            elif self.sel == 1:
                self.out.next = self.in2
            elif self.sel == 2:
                self.out.next = self.in3
            else:
                self.out.next = self.in4

        return rtl
# Local Variables:
# flycheck-flake8-maximum-line-length: 120
# flycheck-flake8rc: ".flake8rc"
# End: