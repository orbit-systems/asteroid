const instr = @import("instructions.zig");
const Instruction = instr.InstructionDesc;

pub const system_control = [_]Instruction{
    .{ .mnemonic = "int", .opcode = 0x01, .encoding = .f, .info = 0x0 },
    .{ .mnemonic = "iret", .opcode = 0x01, .encoding = .f, .info = 0x1 },
    .{ .mnemonic = "ires", .opcode = 0x01, .encoding = .f, .info = 0x2 },
    .{ .mnemonic = "usr", .opcode = 0x01, .encoding = .f, .info = 0x3 },
};

pub const input_output = [_]Instruction{
    .{ .mnemonic = "outr", .opcode = 0x02, .encoding = .m },
    .{ .mnemonic = "outi", .opcode = 0x03, .encoding = .f },
    .{ .mnemonic = "inr", .opcode = 0x04, .encoding = .m },
    .{ .mnemonic = "ini", .opcode = 0x05, .encoding = .f },
};

const BC = instr.BranchCondition;
pub const control_flow = [_]Instruction{
    .{ .mnemonic = "jal", .opcode = 0x06, .encoding = .m },
    .{ .mnemonic = "jalr", .opcode = 0x07, .encoding = .m },
    .{ .mnemonic = "ret", .opcode = 0x08, .encoding = .m },
    .{ .mnemonic = "retr", .opcode = 0x09, .encoding = .m },
    .{ .mnemonic = "bra", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.ra) },
    .{ .mnemonic = "beq", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.eq) },
    .{ .mnemonic = "bez", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.ez) },
    .{ .mnemonic = "blt", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.lt) },
    .{ .mnemonic = "ble", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.le) },
    .{ .mnemonic = "bltu", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.ltu) },
    .{ .mnemonic = "bleu", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.leu) },
    .{ .mnemonic = "bne", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.ne) },
    .{ .mnemonic = "bnz", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.nz) },
    .{ .mnemonic = "bge", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.ge) },
    .{ .mnemonic = "bgt", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.gt) },
    .{ .mnemonic = "bgeu", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.geu) },
    .{ .mnemonic = "bgtu", .opcode = 0x0a, .encoding = .b, .info = @intFromEnum(BC.gtu) },
};

pub const stack = [_]Instruction{
    .{ .mnemonic = "push", .opcode = 0x0b, .encoding = .m },
    .{ .mnemonic = "pop", .opcode = 0x0c, .encoding = .m },
    .{ .mnemonic = "enter", .opcode = 0x0d, .encoding = .b },
    .{ .mnemonic = "leave", .opcode = 0x0e, .encoding = .b },
};

pub fn encoding(opcode: u8) instr.Encoding {
    return switch (opcode) {
        0x01 => .f,
        0x02 => .m,
        0x03 => .f,
        0x04 => .m,
        0x06...0x09 => .m,
        0x0a => .b,
        0x0b, 0x0c => .m,
        0x0d, 0x0e => .b,
        0x10 => .f,
        0x11...0x1b => .e,
        0x1e => .m,
        0x1f => .f,
        0x20...0x3f => if (opcode & 0x01 == 1) .m else .r,
        0x40...0x4e => .e,
        else => .inv,
    };
}

test {
    _ = system_control;
    _ = input_output;
    _ = control_flow;
    _ = stack;
    _ = &branch;
}

// Instruction encoding helpers

const Reg = instr.Reg;

inline fn f(op: u8, imm: u16, func: u4, rde: Reg) instr.F {
    return .{
        .opcode = op,
        .imm = imm,
        .func = func,
        .rde = rde,
    };
}

inline fn b(op: u8, imm: u20, func: u4) instr.B {
    return .{
        .opcode = op,
        .imm = imm,
        .func = func,
    };
}

inline fn r(op: u8, imm: u12, r1: Reg, r2: Reg, rd: Reg) instr.R {
    return .{
        .opcode = op,
        .imm = imm,
        .rs1 = r1,
        .rs2 = r2,
        .rde = rd,
    };
}

pub inline fn nop() u32 {
    return addr(.rz, .rz, .rz);
}

pub inline fn addr(r1: Reg, r2: Reg, rd: Reg) u32 {
    return @bitCast(r(0x20, r1, r2, rd));
}

pub inline fn int(imm8: u8) u32 {
    return @bitCast(f(0x01, imm8, 0x00, .rz));
}

pub inline fn iret() u32 {
    return @bitCast(instr.F{
        .opcode = 0x01,
        .func = 0x01,
        .imm = 0,
        .rde = .rz,
    });
}

pub inline fn ires() u32 {
    return @bitCast(instr.F{
        .opcode = 0x01,
        .func = 0x02,
        .imm = 0,
        .rde = .rz,
    });
}

pub inline fn usr(rd: instr.Reg) u32 {
    return @bitCast(instr.F{
        .opcode = 0x01,
        .func = 0x03,
        .imm = 0,
        .rde = rd,
    });
}

pub inline fn outi(rd: instr.Reg, imm: u16) u32 {
    return @bitCast(f(0x03, imm, 0, rd));
}

pub inline fn branch(cond: BC, address: u20) u32 {
    return @bitCast(b(0x0a, address, @intFromEnum(cond)));
}

pub inline fn ldi(rd: instr.Reg, imm: u16) u32 {
    return @bitCast(f(0x10, imm, 1, rd));
}
