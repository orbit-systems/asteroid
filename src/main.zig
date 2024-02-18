const std = @import("std");
const root = @import("root.zig");
const CPU = root.CPU;
const IVT = root.IVT;
const aphelion = @import("aphelion.zig");
const log = std.log.scoped(.main_loop);

fn assemble(memory: []u8, idx: usize, instr: u32) void {
    std.mem.writeInt(u32, memory[idx..][0..4], instr, .little);
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const alloc = gpa.allocator();
    const memory = try alloc.alloc(u8, 30_000);
    defer alloc.free(memory);
    @memset(memory, 0xFF);

    // var idx: usize = 0;
    // encode(
    //     &memory,
    //     idx,
    //     .{
    //         .f = .{
    //             .opcode = 0x10,
    //             .imm = 100,
    //             .func = 1,
    //             .rde = Reg.ra,
    //         },
    //     },
    // );
    // idx += 4;
    // std.mem.writeInt(u32, (&memory)[idx..][0..4], aphelion.int(2), .little);
    // std.mem.writeInt(u32, (&memory)[16..][0..4], aphelion.iret(), .little);
    // std.mem.writeInt(u32, memory[16..][0..4], aphelion.outi(0xFF), .little);
    assemble(memory, 16, aphelion.ldi(.ra, 0xFF));
    assemble(memory, 20, aphelion.outi(.ra, 0xFF));
    assemble(memory, 24, aphelion.branch(.ra, @bitCast(@as(i20, -1))));

    var ivt: IVT = .{};
    var cpu = CPU.init(memory, &ivt);
    while (true) {
        cpu.run();
        log.debug("OUT: {} IN: {}\nPORT: {x} DATA: {x}", .{ cpu.out_pin, cpu.in_pin, cpu.port, cpu.data_bus });
        if (cpu.out_pin) {
            switch (cpu.port) {
                ports.interrupt_controller => {
                    switch (ivt.state) {
                        .undef => {
                            if (cpu.data_bus == 0) {
                                ivt.state = .primed;
                            }
                        },
                        .primed => {
                            ivt.addr = cpu.data_bus;
                            ivt.state = .undef;
                        },
                    }
                    cpu.out_pin = false;
                },
                // non-standard: writing to port 0xFF is a halt
                0xFF => break,
                else => {},
            }
        }
        if (cpu.in_pin) {
            switch (cpu.port) {
                else => cpu.data_bus = 0xFF,
            }
            cpu.in_pin = false;
        }
    }
}

const ports = struct {
    const interrupt_controller = 0;
    const io_controller = 1;
    const mmu = 2;
    const system_timer = 3;
};
