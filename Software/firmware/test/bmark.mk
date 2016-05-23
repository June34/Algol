# Copyright (c) 2016 Angel Terrones (<angelterrones@gmail.com>)
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

test_c_src = \
	test_main.c \
	syscalls.c \

test_riscv_src = \
	crt.S \

test_c_objs		= $(patsubst %.c, %.o, $(test_c_src))
test_riscv_objs = $(patsubst %.S, %.o, $(test_riscv_src))

test_riscv_bin = test.riscv
$(test_riscv_bin): $(test_c_objs) $(test_riscv_objs)
	$(RISCV_LINK) $(test_c_objs) $(tests_riscv_objs) -o $(test_riscv_bin) $(RISCV_LINK_OPTS)

junk += $(test_c_objs) $(test_riscv_objs) $(test_riscv_bin)
