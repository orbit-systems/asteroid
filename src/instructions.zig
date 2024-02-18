const std = @import("std");

pub const InstructionDesc = struct {
    mnemonic: []const u8,
    opcode: u8,
    encoding: Encoding,
    info: ?u4 = null,
};

pub const Reg = enum(u4) {
    rz,
    ra,
    rb,
    rc,
    rd,
    re,
    rf,
    rg,
    rh,
    ri,
    rj,
    rk,
    ip,
    sp,
    fp,
    st,
};

pub const ST = packed struct(u64) {
    sign: bool,
    zero: bool,
    carry_borrow: bool,
    carry_borrow_unsigned: bool,
    equal: bool,
    less: bool,
    less_unsigned: bool,
    mode: bool,
    unused: u23,
    ext_f: bool,
    ci: u32,
};

pub const Encoding = enum { b, f, m, r, e, inv };

pub const Instruction = union(Encoding) {
    b: B,
    f: F,
    m: M,
    r: R,
    e: E,
    inv: u32,
};

pub const B = packed struct(u32) {
    opcode: u8,
    imm: u20,
    func: u4,
};

pub const BranchCondition = enum(u4) {
    ra,
    eq,
    ez,
    lt,
    le,
    ltu,
    leu,
    ne,
    nz,
    ge,
    gt,
    geu,
    gtu,
};

pub const F = packed struct(u32) {
    opcode: u8,
    imm: u16,
    func: u4,
    rde: Reg,
};

pub const M = packed struct(u32) {
    opcode: u8,
    imm: u16,
    rs1: Reg,
    rde: Reg,
};

pub const R = packed struct(u32) {
    opcode: u8,
    imm: u12,
    rs2: Reg,
    rs1: Reg,
    rde: Reg,
};

pub const E = packed struct(u32) {
    opcode: u8,
    imm: u8,
    func: u4,
    rs2: Reg,
    rs1: Reg,
    rde: Reg,
};

test "encodings" {
    _ = B{ .opcode = 0, .imm = 0, .func = 0 };
    _ = F{ .opcode = 0, .imm = 0, .func = 0, .rde = .ra };
    _ = M{ .opcode = 0, .imm = 0, .rs1 = .rz, .rde = .ra };
    _ = R{ .opcode = 0, .imm = 0, .rs1 = .ra, .rs2 = .rb, .rde = .rz };
    _ = E{ .opcode = 0, .imm = 0, .func = 0, .rs1 = .ra, .rs2 = .rb, .rde = .rz };
}

test {
    _ = InstructionDesc{ .opcode = 0, .mnemonic = "", .encoding = .b };
    _ = Instruction{ .b = .{ .opcode = 0, .imm = 0, .func = 0 } };
    _ = std.mem.zeroes(ST);
}
