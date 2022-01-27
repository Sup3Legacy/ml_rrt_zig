const std = @import("std");

pub fn main() anyerror!void {
    const model = @import("model.zig");
    _ = model.JointTree;
    _ = model.Scene.update;
    _ = model.Object.to_rects;
    std.log.info("All your codebase are belong to us.", .{});
}
