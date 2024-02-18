const std = @import("std");
const instructions = @import("instructions.zig");
const aphelion = @import("aphelion.zig");
const testing = std.testing;

const Reg = instructions.Reg;

pub const IVT = struct {
    addr: u64 = 0,
    state: enum { undef, primed } = .undef,
};

pub const CPU = struct {
    registers: [16]u64 = .{0} ** 16,
    memory: []u8,
    interrupt_queue: IQ = IQ.init(),
    interrupt_ret: u64 = 0,
    interrupt_ret_mode: Mode = .kernel,
    mode: Mode = .kernel,
    ivt_base_address: u64 = 0,
    port: u8 = 0,
    out_pin: bool = false,
    in_pin: bool = false,
    data_bus: u64 = 0,
    ivt_conn: *IVT,

    const IQ = std.fifo.LinearFifo(u8, .{ .Static = 32 });
    const Mode = enum { kernel, user };

    pub fn init(memory: []u8, ivt: *IVT) CPU {
        return .{ .memory = memory, .ivt_conn = ivt };
    }

    pub fn enableFloat(cpu: *CPU) void {
        cpu.setFlag(flags.ext_f);
    }

    pub fn disableFloat(cpu: *CPU) void {
        cpu.unsetFlag(flags.ext_f);
    }

    pub fn run(cpu: *CPU) void {
        cpu.registers[rz] = 0;
        if (cpu.in_pin) return;
        if (cpu.interrupt_queue.count > 0) {
            {
                log.err("Interrupt {x}", .{cpu.interrupt_queue.peekItem(0)});
                log.err("Instruction {X:0>8}", .{@as(u32, @truncate(cpu.registers[st] >> 32))});
                for (0..16) |idx| {
                    log.err("CPU[{s}] = {X:0>16}", .{ @tagName(@as(Reg, @enumFromInt(idx))), cpu.registers[idx] });
                }
                if (cpu.interrupt_queue.peekItem(0) == interrupts.interrupt_overflow) @panic("Interrupt");
            }
            cpu.interrupt_ret_mode = cpu.mode;
            cpu.mode = .kernel;
            cpu.interrupt_ret = cpu.registers[ip];
            cpu.registers[ip] = cpu.ivt_conn.addr + 8 * cpu.interrupt_queue.peekItem(0);
        }
        const instr = std.mem.readInt(u32, cpu.memory[cpu.registers[ip]..][0..4], .little);
        {
            var st_flags: instructions.ST = @bitCast(cpu.registers[st]);
            st_flags.ci = instr;
            cpu.registers[st] = @bitCast(st_flags);
        }
        const op: u8 = @truncate(instr);
        const in: instructions.Instruction =
            switch (aphelion.encoding(op)) {
            .b => .{ .b = @bitCast(instr) },
            .f => .{ .f = @bitCast(instr) },
            .m => .{ .m = @bitCast(instr) },
            .r => .{ .r = @bitCast(instr) },
            .e => .{ .e = @bitCast(instr) },
            .inv => return cpu.int(interrupts.invalid_operation),
        };
        log.info("[{X:0>16}] Instruction: {} ({X:0>8})", .{ cpu.registers[ip], in, instr });
        cpu.registers[ip] += 4;
        switch (op) {
            0x00 => unreachable,
            0x01 => switch (in.f.func) {
                // int
                0x00 => {
                    const idx: u8 = @truncate(in.f.imm);
                    cpu.int(idx);
                },
                // iret
                0x01 => {
                    cpu.registers[ip] = cpu.interrupt_ret;
                    cpu.mode = cpu.interrupt_ret_mode;
                    cpu.interrupt_queue.discard(1);
                },
                // ires
                0x02 => {
                    cpu.mode = cpu.interrupt_ret_mode;
                    cpu.interrupt_queue.discard(1);
                },
                // usr
                0x03 => {
                    const addr = cpu.registers[@intFromEnum(in.f.rde)];
                    // jump to addr in rde
                    cpu.registers[ip] = addr;
                    cpu.mode = .user;
                },
                else => unreachable,
            },
            // outr
            0x02 => {
                if (cpu.mode == .user) return cpu.inv();
                cpu.port = @truncate(cpu.registers[@intFromEnum(in.m.rde)]);
                cpu.data_bus = cpu.registers[@intFromEnum(in.m.rs1)];
                cpu.out_pin = true;
            },
            // outi
            0x03 => {
                if (cpu.mode == .user) return cpu.inv();
                cpu.port = @truncate(in.f.imm);
                cpu.data_bus = cpu.registers[@intFromEnum(in.f.rde)];
                cpu.out_pin = true;
            },
            // inr
            0x04 => {
                if (cpu.mode == .user) return cpu.inv();
                cpu.port = @truncate(cpu.registers[@intFromEnum(in.m.rs1)]);
                cpu.data_bus = cpu.registers[@intFromEnum(in.m.rde)];
                cpu.in_pin = true;
            },
            // ini
            0x05 => {
                if (cpu.mode == .user) return cpu.inv();
                cpu.port = @truncate(in.f.imm);
                cpu.data_bus = cpu.registers[@intFromEnum(in.f.rde)];
                cpu.in_pin = true;
            },
            // jal
            0x06 => {
                // push ip, ip <- rs + 4 * (i64)imm16
                cpu.push(cpu.registers[ip]);
                cpu.registers[ip] = @intCast(@as(i64, @bitCast(cpu.registers[@intFromEnum(in.m.rs1)])) + 4 * @as(i64, in.m.imm));
            },
            // jalr
            0x07 => {
                // rd <- ip, ip <- rs + 4 * (i64)imm16
                if (in.m.rde == .st) return cpu.int(2);
                cpu.registers[@intFromEnum(in.m.rde)] = cpu.registers[ip];
                cpu.registers[ip] = @intCast(@as(i64, @bitCast(cpu.registers[@intFromEnum(in.m.rs1)])) + 4 * @as(i64, in.m.imm));
            },
            // ret
            0x08 => {
                cpu.registers[ip] = cpu.pop();
            },
            // retr
            0x09 => {
                cpu.registers[ip] = cpu.registers[@intFromEnum(in.m.rs1)];
            },
            // branch
            0x0a => {
                const addr: u64 = @intCast(@as(i64, @bitCast(cpu.registers[ip])) + 4 * @as(i64, in.b.imm));
                const st_flags: instructions.ST = @bitCast(cpu.registers[st]);
                const BC = instructions.BranchCondition;
                switch (@as(BC, @enumFromInt(in.b.func))) {
                    .ra => cpu.registers[ip] = addr,
                    .eq => if (st_flags.equal) {
                        cpu.registers[ip] = addr;
                    },
                    .ez => if (st_flags.zero) {
                        cpu.registers[ip] = addr;
                    },
                    .lt => if (st_flags.less) {
                        cpu.registers[ip] = addr;
                    },
                    .le => if (st_flags.less or st_flags.equal) {
                        cpu.registers[ip] = addr;
                    },
                    .ltu => if (st_flags.less_unsigned) {
                        cpu.registers[ip] = addr;
                    },
                    .leu => if (st_flags.less_unsigned or st_flags.equal) {
                        cpu.registers[ip] = addr;
                    },
                    .ne => if (!st_flags.equal) {
                        cpu.registers[ip] = addr;
                    },
                    .nz => if (!st_flags.zero) {
                        cpu.registers[ip] = addr;
                    },
                    .ge => if (!st_flags.less) {
                        cpu.registers[ip] = addr;
                    },
                    .gt => if (!st_flags.less and !st_flags.equal) {
                        cpu.registers[ip] = addr;
                    },
                    .geu => if (!st_flags.less_unsigned) {
                        cpu.registers[ip] = addr;
                    },
                    .gtu => if (!st_flags.less_unsigned and !st_flags.equal) {
                        cpu.registers[ip] = addr;
                    },
                }
            },
            // push
            0x0b => {
                cpu.push(cpu.registers[@intFromEnum(in.m.rs1)]);
            },
            // pop
            0x0c => {
                cpu.registers[@intFromEnum(in.m.rde)] = cpu.pop();
            },
            // enter
            0x0d => cpu.todo(op, in),
            // leave
            0x0e => cpu.todo(op, in),
            // load
            0x10 => {
                switch (in.f.func) {
                    0 => std.mem.writeInt(u16, std.mem.asBytes(&cpu.registers[@intFromEnum(in.f.rde)])[0..2], in.f.imm, .little),
                    1 => cpu.registers[@intFromEnum(in.f.rde)] = @intCast(in.f.imm),
                    2 => std.mem.writeInt(u16, std.mem.asBytes(&cpu.registers[@intFromEnum(in.f.rde)])[2..4], in.f.imm, .little),
                    3 => cpu.registers[@intFromEnum(in.f.rde)] = @as(u64, @intCast(in.f.imm)) << 16,
                    4 => std.mem.writeInt(u16, std.mem.asBytes(&cpu.registers[@intFromEnum(in.f.rde)])[4..6], in.f.imm, .little),
                    5 => cpu.registers[@intFromEnum(in.f.rde)] = @as(u64, @intCast(in.f.imm)) << 32,
                    6 => std.mem.writeInt(u16, std.mem.asBytes(&cpu.registers[@intFromEnum(in.f.rde)])[6..8], in.f.imm, .little),
                    7 => cpu.registers[@intFromEnum(in.f.rde)] = @as(u64, @intCast(in.f.imm)) << 48,
                    else => unreachable,
                }
            },
            // extended load
            0x11...0x1b => {
                cpu.todo(op, in);
            },
            // bswp DEPRECATED
            0x1c => {
                cpu.registers[@intFromEnum(in.r.rs2)] = @byteSwap(cpu.registers[@intFromEnum(in.r.rs1)]);
            },
            // xch DEPRECATED
            0x1d => {
                std.mem.swap(u64, &cpu.registers[@intFromEnum(in.m.rde)], &cpu.registers[@intFromEnum(in.m.rs1)]);
            },
            // cmpr
            0x1e => {
                const r1 = cpu.registers[@intFromEnum(in.m.rde)];
                const r2 = cpu.registers[@intFromEnum(in.m.rs1)];
                var st_flags: instructions.ST = @bitCast(cpu.registers[st]);
                defer cpu.registers[st] = @bitCast(st_flags);
                const cmpu = std.math.order(r1, r2);
                switch (cmpu) {
                    .gt => {
                        st_flags.equal = false;
                        st_flags.less_unsigned = false;
                    },
                    .lt => {
                        st_flags.equal = false;
                        st_flags.less_unsigned = true;
                    },
                    .eq => {
                        st_flags.equal = true;
                        st_flags.less = false;
                        st_flags.less_unsigned = false;
                    },
                }
                const r1i: i64 = @bitCast(r1);
                const r2i: i64 = @bitCast(r2);
                const cmpi = std.math.order(r1i, r2i);
                if (cmpi == .lt) {
                    st_flags.less = true;
                } else {
                    st_flags.less = false;
                }
                st_flags.sign = r1i < 0;
            },
            // cmpi
            0x1f => {
                var r1 = cpu.registers[@intFromEnum(in.f.rde)];
                var r2 = se(in.f.imm);
                if (in.f.func == 0x1) std.mem.swap(u64, &r1, &r2);
                var st_flags: instructions.ST = @bitCast(cpu.registers[st]);
                defer cpu.registers[st] = @bitCast(st_flags);

                const cmpu = std.math.order(r1, r2);
                switch (cmpu) {
                    .gt => {
                        st_flags.equal = false;
                        st_flags.less_unsigned = false;
                    },
                    .lt => {
                        st_flags.equal = false;
                        st_flags.less_unsigned = true;
                    },
                    .eq => {
                        st_flags.equal = true;
                        st_flags.less = false;
                        st_flags.less_unsigned = false;
                    },
                }

                const r1i: i64 = @bitCast(r1);
                const r2i: i64 = @bitCast(r2);
                const cmpi = std.math.order(r1i, r2i);
                if (cmpi == .lt) {
                    st_flags.less = true;
                } else {
                    st_flags.less = false;
                }
                st_flags.sign = r1i < 0;
            },
            // addr
            0x20 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                const ai: i64 = @bitCast(a);
                const bi: i64 = @bitCast(b);
                const res, const overflow = @addWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.r.rde)] = res; // + @intFromBool(cpu.getFlag(flags.carry_borrow));
                if (overflow == 1) cpu.setFlag(flags.carry_borrow_unsigned);

                _, const sover = @addWithOverflow(ai, bi);
                if (sover == 1) cpu.setFlag(flags.carry_borrow);
            },
            // addi
            0x21 => {
                const a = cpu.registers[@intFromEnum(in.m.rs1)];
                const b = se(in.m.imm);
                const ai: i64 = @bitCast(a);
                const bi: i64 = @bitCast(b);
                const res, const overflow = @addWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.m.rde)] = res; // + @intFromBool(cpu.getFlag(flags.carry_borrow));
                if (overflow == 1) cpu.setFlag(flags.carry_borrow_unsigned);

                _, const sover = @addWithOverflow(ai, bi);
                if (sover == 1) cpu.setFlag(flags.carry_borrow);
            },
            // subr
            0x22 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                const ai: i64 = @bitCast(a);
                const bi: i64 = @bitCast(b);
                const res, const overflow = @subWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.r.rde)] = res; // - @intFromBool(cpu.getFlag(flags.carry_borrow));
                if (overflow == 1) cpu.setFlag(flags.carry_borrow_unsigned);

                _, const sover = @subWithOverflow(ai, bi);
                if (sover == 1) cpu.setFlag(flags.carry_borrow);
            },
            // subr
            0x23 => {
                const a = cpu.registers[@intFromEnum(in.m.rs1)];
                const b = se(in.m.imm);
                const ai: i64 = @bitCast(a);
                const bi: i64 = @bitCast(b);
                const res, const overflow = @subWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.m.rde)] = res; // - @intFromBool(cpu.getFlag(flags.carry_borrow));
                if (overflow == 1) cpu.setFlag(flags.carry_borrow_unsigned);

                _, const sover = @subWithOverflow(ai, bi);
                if (sover == 1) cpu.setFlag(flags.carry_borrow);
            },
            // imulr
            0x24 => {
                const a: i64 = @bitCast(cpu.registers[@intFromEnum(in.r.rs1)]);
                const b: i64 = @bitCast(cpu.registers[@intFromEnum(in.r.rs2)]);
                const res, _ = @mulWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.r.rde)] = @bitCast(res);
            },
            // imuli
            0x25 => {
                const a: i64 = @bitCast(cpu.registers[@intFromEnum(in.m.rs1)]);
                const b: i64 = @bitCast(se(in.m.imm));
                const res, _ = @mulWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.m.rde)] = @bitCast(res);
            },
            // idivr
            0x26 => {
                const a: i64 = @bitCast(cpu.registers[@intFromEnum(in.r.rs1)]);
                const b: i64 = @bitCast(cpu.registers[@intFromEnum(in.r.rs2)]);
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.r.rde)] = @bitCast(@divTrunc(a, b));
            },
            // idivi
            0x27 => {
                const a: i64 = @bitCast(cpu.registers[@intFromEnum(in.m.rs1)]);
                const b: i64 = @bitCast(se(in.m.imm));
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.m.rde)] = @bitCast(@divTrunc(a, b));
            },
            // umulr
            0x28 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                const res, _ = @mulWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.r.rde)] = res;
            },
            // umuli
            0x29 => {
                const a = cpu.registers[@intFromEnum(in.m.rs1)];
                const b = se(in.m.imm);
                const res, _ = @mulWithOverflow(a, b);
                cpu.registers[@intFromEnum(in.m.rde)] = res;
            },
            // udivr
            0x2a => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.r.rde)] = a / b;
            },
            // udivi
            0x2b => {
                const a = cpu.registers[@intFromEnum(in.m.rs1)];
                const b = se(in.m.imm);
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.m.rde)] = a / b;
            },
            // remr
            0x2c => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.r.rde)] = @rem(a, b);
            },
            // remi
            0x2d => {
                const a: i64 = @bitCast(cpu.registers[@intFromEnum(in.m.rs1)]);
                const b: i64 = @bitCast(se(in.m.imm));
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.r.rde)] = @bitCast(@rem(a, b));
            },
            // modr
            0x2e => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.r.rde)] = @mod(a, b);
            },
            // modi
            0x2f => {
                const a: i64 = @bitCast(cpu.registers[@intFromEnum(in.m.rs1)]);
                const b: i64 = @bitCast(se(in.m.imm));
                if (b == 0) return cpu.int(interrupts.divide_by_zero);
                cpu.registers[@intFromEnum(in.r.rde)] = @bitCast(@mod(a, b));
            },
            // andr
            0x30 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                cpu.registers[@intFromEnum(in.r.rde)] = a & b;
            },
            // andi
            0x31 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b: u64 = @intCast(in.m.imm);
                cpu.registers[@intFromEnum(in.r.rde)] = a & b;
            },
            // orr
            0x32 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                cpu.registers[@intFromEnum(in.r.rde)] = a | b;
            },
            // ori
            0x33 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b: u64 = @intCast(in.m.imm);
                cpu.registers[@intFromEnum(in.r.rde)] = a | b;
            },
            // norr
            0x34 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                cpu.registers[@intFromEnum(in.r.rde)] = ~(a | b);
            },
            // nori
            0x35 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b: u64 = @intCast(in.m.imm);
                cpu.registers[@intFromEnum(in.r.rde)] = ~(a | b);
            },
            // xorr
            0x36 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                cpu.registers[@intFromEnum(in.r.rde)] = a ^ b;
            },
            // xori
            0x37 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b: u64 = @intCast(in.m.imm);
                cpu.registers[@intFromEnum(in.r.rde)] = a ^ b;
            },
            // shlr
            0x38 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = @shlExact(a, @truncate(b));
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = 0;
                }
            },
            // shli
            0x39 => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = in.m.imm;
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = @shlExact(a, @truncate(b));
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = 0;
                }
            },
            // asrr
            0x3a => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                const fill: u64 = if (a & @as(u64, 1 << 63) == 1) @bitCast(@as(i64, -1)) else 0;
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = a >> @truncate(b);
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = fill;
                }
            },
            // asri
            0x3b => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = in.m.imm;
                const fill: u64 = if (a & @as(u64, 1 << 63) == 1) @bitCast(@as(i64, -1)) else 0;
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = a >> @truncate(b);
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = fill;
                }
            },
            // lsrr
            0x3c => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = a >> @truncate(b);
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = 0;
                }
            },
            // lsri
            0x3d => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = in.m.imm;
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = a >> @truncate(b);
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = 0;
                }
            },
            // bitr
            0x3e => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b = cpu.registers[@intFromEnum(in.r.rs2)];
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = @intFromBool(1 == a >> @truncate(b));
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = 0;
                }
            },
            // biti
            0x3f => {
                const a = cpu.registers[@intFromEnum(in.r.rs1)];
                const b: u64 = @intCast(in.m.imm);
                if (b < 64) {
                    cpu.registers[@intFromEnum(in.r.rde)] = @intFromBool(1 == a >> @truncate(b));
                } else {
                    cpu.registers[@intFromEnum(in.r.rde)] = 0;
                }
            },
            // fcmp
            0x40 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                var st_flags: instructions.ST = @bitCast(cpu.registers[st]);
                defer cpu.registers[st] = @bitCast(st_flags);
                switch (in.e.func) {
                    0 => {
                        const r1 = fflow(f16, cpu.registers[@intFromEnum(in.e.rs1)]);
                        const r2 = fflow(f16, cpu.registers[@intFromEnum(in.e.rs2)]);
                        const ord = std.math.order(r1, r2);
                        switch (ord) {
                            .lt => {
                                st_flags.less = true;
                                st_flags.equal = false;
                            },
                            .gt => {
                                st_flags.less = false;
                                st_flags.equal = false;
                            },
                            .eq => {
                                st_flags.less = false;
                                st_flags.equal = true;
                            },
                        }
                        st_flags.sign = r1 < 0.0;
                    },
                    1 => {
                        const r1 = fflow(f32, cpu.registers[@intFromEnum(in.e.rs1)]);
                        const r2 = fflow(f32, cpu.registers[@intFromEnum(in.e.rs2)]);
                        const ord = std.math.order(r1, r2);
                        switch (ord) {
                            .lt => {
                                st_flags.less = true;
                                st_flags.equal = false;
                            },
                            .gt => {
                                st_flags.less = false;
                                st_flags.equal = false;
                            },
                            .eq => {
                                st_flags.less = false;
                                st_flags.equal = true;
                            },
                        }
                        st_flags.sign = r1 < 0.0;
                    },
                    2 => {
                        const r1 = fflow(f64, cpu.registers[@intFromEnum(in.e.rs1)]);
                        const r2 = fflow(f64, cpu.registers[@intFromEnum(in.e.rs2)]);
                        const ord = std.math.order(r1, r2);
                        switch (ord) {
                            .lt => {
                                st_flags.less = true;
                                st_flags.equal = false;
                            },
                            .gt => {
                                st_flags.less = false;
                                st_flags.equal = false;
                            },
                            .eq => {
                                st_flags.less = false;
                                st_flags.equal = true;
                            },
                        }
                        st_flags.sign = r1 < 0.0;
                    },
                    else => return cpu.inv(),
                }
            },
            // fto
            0x41 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r = cpu.registers[@intFromEnum(in.e.rs1)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => flow(f16, r),
                    1 => flow(f32, r),
                    2 => flow(f64, r),
                    else => return cpu.inv(),
                };
            },
            // ffrom
            0x42 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r = cpu.registers[@intFromEnum(in.e.rs1)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => deflow(f16, r),
                    1 => deflow(f32, r),
                    2 => deflow(f64, r),
                    else => return cpu.inv(),
                };
            },
            // fneg
            0x43 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r = cpu.registers[@intFromEnum(in.e.rs1)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => neg(f16, r),
                    1 => neg(f32, r),
                    2 => neg(f64, r),
                    else => return cpu.inv(),
                };
            },
            // fabs
            0x44 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r = cpu.registers[@intFromEnum(in.e.rs1)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => abs(f16, r),
                    1 => abs(f32, r),
                    2 => abs(f64, r),
                    else => return cpu.inv(),
                };
            },
            // fadd
            0x45 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r1 = cpu.registers[@intFromEnum(in.e.rs1)];
                const r2 = cpu.registers[@intFromEnum(in.e.rs2)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => fdeflow(f16, fflow(f16, r1) + fflow(f16, r2)),
                    1 => fdeflow(f32, fflow(f32, r1) + fflow(f32, r2)),
                    2 => fdeflow(f64, fflow(f64, r1) + fflow(f64, r2)),
                    else => return cpu.inv(),
                };
            },
            // fsub
            0x46 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r1 = cpu.registers[@intFromEnum(in.e.rs1)];
                const r2 = cpu.registers[@intFromEnum(in.e.rs2)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => fdeflow(f16, fflow(f16, r1) - fflow(f16, r2)),
                    1 => fdeflow(f32, fflow(f32, r1) - fflow(f32, r2)),
                    2 => fdeflow(f64, fflow(f64, r1) - fflow(f64, r2)),
                    else => return cpu.inv(),
                };
            },
            // fmul
            0x47 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r1 = cpu.registers[@intFromEnum(in.e.rs1)];
                const r2 = cpu.registers[@intFromEnum(in.e.rs2)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => fdeflow(f16, fflow(f16, r1) * fflow(f16, r2)),
                    1 => fdeflow(f32, fflow(f32, r1) * fflow(f32, r2)),
                    2 => fdeflow(f64, fflow(f64, r1) * fflow(f64, r2)),
                    else => return cpu.inv(),
                };
            },
            // fdiv
            0x48 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r1 = cpu.registers[@intFromEnum(in.e.rs1)];
                const r2 = cpu.registers[@intFromEnum(in.e.rs2)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => fdeflow(f16, fflow(f16, r1) / fflow(f16, r2)),
                    1 => fdeflow(f32, fflow(f32, r1) / fflow(f32, r2)),
                    2 => fdeflow(f64, fflow(f64, r1) / fflow(f64, r2)),
                    else => return cpu.inv(),
                };
            },
            // fma
            0x49 => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r1 = cpu.registers[@intFromEnum(in.e.rs1)];
                const r2 = cpu.registers[@intFromEnum(in.e.rs2)];
                const rd = cpu.registers[@intFromEnum(in.e.rde)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => fdeflow(f16, @mulAdd(f16, fflow(f16, r1), fflow(f16, r2), fflow(f16, rd))),
                    1 => fdeflow(f32, @mulAdd(f32, fflow(f32, r1), fflow(f32, r2), fflow(f32, rd))),
                    2 => fdeflow(f64, @mulAdd(f64, fflow(f64, r1), fflow(f64, r2), fflow(f64, rd))),
                    else => return cpu.inv(),
                };
            },
            // fsqrt
            0x4a => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r = cpu.registers[@intFromEnum(in.e.rs1)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => sqrt(f16, r),
                    1 => sqrt(f32, r),
                    2 => sqrt(f64, r),
                    else => return cpu.inv(),
                };
            },
            // fmin
            0x4b => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r1 = cpu.registers[@intFromEnum(in.e.rs1)];
                const r2 = cpu.registers[@intFromEnum(in.e.rs2)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => fdeflow(f16, @min(fflow(f16, r1), fflow(f16, r2))),
                    1 => fdeflow(f32, @min(fflow(f32, r1), fflow(f32, r2))),
                    2 => fdeflow(f64, @min(fflow(f64, r1), fflow(f64, r2))),
                    else => return cpu.inv(),
                };
            },
            // fmax
            0x4c => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r1 = cpu.registers[@intFromEnum(in.e.rs1)];
                const r2 = cpu.registers[@intFromEnum(in.e.rs2)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => fdeflow(f16, @max(fflow(f16, r1), fflow(f16, r2))),
                    1 => fdeflow(f32, @max(fflow(f32, r1), fflow(f32, r2))),
                    2 => fdeflow(f64, @max(fflow(f64, r1), fflow(f64, r2))),
                    else => return cpu.inv(),
                };
            },
            // fsat
            0x4d => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r = cpu.registers[@intFromEnum(in.e.rs1)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0 => ceil(f16, r),
                    1 => ceil(f32, r),
                    2 => ceil(f64, r),
                    else => return cpu.inv(),
                };
            },
            // fcnv
            0x4e => {
                if (!cpu.getFlag(flags.ext_f)) {
                    return cpu.int(interrupts.invalid_operation);
                }
                const r = cpu.registers[@intFromEnum(in.e.rs1)];
                cpu.registers[@intFromEnum(in.e.rde)] = switch (in.e.func) {
                    0b0000 => fconv(f16, f16, r),
                    0b0001 => fconv(f16, f32, r),
                    0b0010 => fconv(f16, f64, r),
                    0b0100 => fconv(f32, f16, r),
                    0b0101 => fconv(f32, f32, r),
                    0b0110 => fconv(f32, f64, r),
                    0b1000 => fconv(f64, f16, r),
                    0b1001 => fconv(f64, f32, r),
                    0b1010 => fconv(f64, f64, r),
                    else => return cpu.inv(),
                };
            },
            else => cpu.todo(op, in),
        }
    }

    inline fn push(cpu: *CPU, value: u64) void {
        cpu.registers[sp] -= 8;
        std.mem.writeInt(
            u64,
            cpu.memory[cpu.registers[sp]..][0..8],
            value,
            .little,
        );
    }

    inline fn pop(cpu: *CPU) u64 {
        const v = std.mem.readInt(
            u64,
            cpu.memory[cpu.registers[sp]..][0..8],
            .little,
        );
        cpu.registers[sp] += 8;
        return v;
    }

    inline fn todo(cpu: *CPU, op: u8, in: instructions.Instruction) void {
        log.warn("TODO: OPCODE {x:0>2} {}", .{ op, in });
        cpu.int(interrupts.invalid_operation);
    }

    inline fn int(cpu: *CPU, idx: u8) void {
        cpu.interrupt_queue.writeItem(idx) catch {
            cpu.interrupt_queue.discard(32);
            cpu.interrupt_queue.writeItem(interrupts.interrupt_overflow) catch unreachable;
        };
    }

    inline fn inv(cpu: *CPU) void {
        cpu.int(interrupts.invalid_operation);
    }

    inline fn getFlag(cpu: *CPU, flag_idx: u6) bool {
        return (cpu.registers[st] & @as(u64, 1 << flag_idx)) == 1;
    }

    inline fn setFlag(cpu: *CPU, flag_idx: u6) void {
        cpu.registers[st] |= @as(u64, 1 << flag_idx);
    }

    inline fn unsetFlag(cpu: *CPU, flag_idx: u6) void {
        cpu.registers[st] &= ~(@as(u64, 1 << flag_idx));
    }

    inline fn se(value: anytype) u64 {
        const ones: u64 = 0xFFFFFFFF;
        if (value < 0) {
            return ones & @abs(value);
        } else {
            return value;
        }
    }

    /// read float value stored in u64
    inline fn fflow(comptime T: type, v: u64) T {
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        const fv: UT = @truncate(v);
        return @bitCast(fv);
    }

    /// store float value into u64
    inline fn fdeflow(comptime T: type, v: T) u64 {
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        return @as(UT, @bitCast(v));
    }

    /// convert v to float and bitcast it to u64
    inline fn flow(comptime T: type, v: u64) u64 {
        const float: T = @floatFromInt(v);
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        return @as(UT, @bitCast(float));
    }

    /// convert v from float type T to i64 (bitcasted to u64)
    inline fn deflow(comptime T: type, v: u64) u64 {
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        const float_bit: UT = @truncate(v);
        const float: T = @bitCast(float_bit);
        return @bitCast(@as(i64, @intFromFloat(float)));
    }

    /// negate float value of type T stored in u64
    inline fn neg(comptime T: type, v: u64) u64 {
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        const float_bit: UT = @truncate(v);
        const float: T = @bitCast(float_bit);
        return @as(UT, @bitCast(-float));
    }

    /// abs float value of type T stored in u64
    inline fn abs(comptime T: type, v: u64) u64 {
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        const float_bit: UT = @truncate(v);
        const float: T = @bitCast(float_bit);
        return @as(UT, @bitCast(@abs(float)));
    }

    /// sqrt float value of type T stored in u64
    inline fn sqrt(comptime T: type, v: u64) u64 {
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        const float_bit: UT = @truncate(v);
        const float: T = @bitCast(float_bit);
        return @as(UT, @bitCast(@sqrt(float)));
    }

    /// ceil float value of type T stored in u64
    inline fn ceil(comptime T: type, v: u64) u64 {
        const UT = std.meta.Int(.unsigned, @typeInfo(T).Float.bits);
        const float_bit: UT = @truncate(v);
        const float: T = @bitCast(float_bit);
        return @as(UT, @bitCast(@ceil(float)));
    }

    /// convert u64-encoded float from type S to type D
    inline fn fconv(comptime S: type, comptime D: type, v: u64) u64 {
        return fdeflow(D, @floatCast(fflow(S, v)));
    }

    const log = std.log.scoped(.cpu);
    const ip = @intFromEnum(Reg.ip);
    const sp = @intFromEnum(Reg.sp);
    const st = @intFromEnum(Reg.st);
    const rz = @intFromEnum(Reg.rz);

    const interrupts = struct {
        const divide_by_zero = 0x00;
        const breakpoint = 0x01;
        const invalid_operation = 0x02;
        const stack_underflow = 0x03;
        const unaligned_access = 0x04;
        const access_violation = 0x05;
        const interrupt_overflow = 0x06;
    };

    const flags = struct {
        const sign = 0;
        const zero = 1;
        const carry_borrow = 2;
        const carry_borrow_unsigned = 3;
        const equal = 4;
        const less = 5;
        const less_unsigned = 6;
        const mode = 7;
        const ext_f = 31;
    };
};

test {
    _ = instructions;
    _ = aphelion;
    // var cpu: CPU = .{ .registers = .{0} ** 16 };
    // cpu.instruction(0x01);
}
