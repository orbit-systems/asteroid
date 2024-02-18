const std = @import("std");
const root = @import("root.zig");
const CPU = root.CPU;
const IVT = root.IVT;
const aphelion = @import("aphelion.zig");

pub fn main() !void {
    var memory: [30000]u8 = undefined;
    @memset(&memory, 0xFF);

    var idx: usize = 0;
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
    idx += 4;
    std.mem.writeInt(u32, (&memory)[idx..][0..4], aphelion.int(2), .little);
    std.mem.writeInt(u32, (&memory)[16..][0..4], aphelion.iret(), .little);

    var ivt: IVT = .{};
    var cpu = CPU.init(&memory, &ivt);
    while (true) {
        cpu.run();
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
