const std = @import("std");
const Allocator = std.mem.Allocator;

pub const Coords = struct {
    theta: f64,
    x: f64,
    y: f64,

    pub fn dir(a: *@This(), b: *@This(), c: *@This()) u8 {
        var disc = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
        if (disc == 0) {
            return 0;
        } else if (disc < 0) {
            return 1;
        } else {
            return 2;
        }
    }
};

pub const Part = union(enum) {
    rect: Rect,
    joint: Joint,
};

pub const Line = struct {
    p1: Coords,
    p2: Coords,

    pub fn intersects(this: *@This(), other: *@This()) void {
        var d1 = this.p1.dir(this.p2, other.p1);
        var d2 = this.p1.dir(this.p2, other.p2);
        var d3 = other.p1.dir(other.p2, this.p1);
        var d4 = other.p1.dir(other.p2, this.p2);

        // We only want to return intersecting collision, not tangenting ones
        return ((d1 != d2) and (d3 != d4));
    }
};

// Anchor is set to (0, 0)
pub const Rect = struct {
    constraints: *JointNode,
    pos: Coords,
    length: f64,
    width: f64,

    pub fn collides(this: *@This(), other: *@This()) bool {
        var points_1 = {
            this.pos;
            Coords{
                .theta = 0,
                .x = this.pos.x + this.length * @sin(this.theta),
                .y = this.pos.y + this.length * @cos(this.theta),
            };
            Coords{
                .theta = 0,
                .x = this.pos.x + this.width * @cos(this.theta),
                .y = this.pos.y - this.length * @sin(this.theta),
            };
            Coords{
                .theta = 0,
                .x = this.pos.x + this.width * @cos(this.theta) + this.length * @sin(this.theta),
                .y = this.pos.y - this.length * @sin(this.theta) + this.length * @cos(this.theta),
            };
        };
        var points_2 = {
            other.pos;
            Coords{
                .other = 0,
                .x = other.pos.x + other.length * @sin(other.theta),
                .y = other.pos.y + other.length * @cos(other.theta),
            };
            Coords{
                .theta = 0,
                .x = other.pos.x + other.width * @cos(other.theta),
                .y = other.pos.y - other.length * @sin(other.theta),
            };
            Coords{
                .theta = 0,
                .x = other.pos.x + other.width * @cos(other.theta) + other.length * @sin(other.theta),
                .y = other.pos.y - other.length * @sin(other.theta) + other.length * @cos(other.theta),
            };
        };
        var lines_1 = {
            Line{
                .p1 = points_1[0],
                .p2 = points_1[1],
            };
            Line{
                .p1 = points_1[1],
                .p2 = points_1[2],
            };
            Line{
                .p1 = points_1[2],
                .p2 = points_1[3],
            };
            Line{
                .p1 = points_1[3],
                .p2 = points_1[0],
            };
        };
        var lines_2 = {
            Line{
                .p1 = points_2[0],
                .p2 = points_2[1],
            };
            Line{
                .p1 = points_2[1],
                .p2 = points_2[2],
            };
            Line{
                .p1 = points_2[2],
                .p2 = points_2[3],
            };
            Line{
                .p1 = points_2[3],
                .p2 = points_2[0],
            };
        };

        var i: usize = 0;
        var j: usize = 0;

        while (i < 4) : (i + 1) {
            j = 0;
            while (j < 4) : (j + 1) {
                if (lines_1[i].intersects(*lines_2[j])) {
                    return true;
                }
            }
        }

        return false;
    }
};

pub const Joint = struct {
    child: Object,
    offset: Coords,
};

const JointNode = struct {
    data: *Joint,
    offset: Coords,
    children: std.ArrayList(*JointNode),
    allocator: Allocator,

    fn push(this: *@This(), item: *Joint) *JointNode {
        var new_node = this.allocator.create(JointNode) catch {
            unreachable;
        };
        new_node.data = item;
        // Doesn't matter if the local total offset if not properly initialized.
        // This will get taken care of on the next forward
        new_node.offset = this.offset;
        new_node.children = std.ArrayList(*JointNode).init(this.allocator);
        new_node.allocator = this.allocator;
        this.children.append(new_node) catch {};
        return new_node;
    }

    fn forward(this: *@This(), offset: Coords) void {
        var add_offset = this.data.offset;
        var new_offset = Coords{
            .theta = offset.theta + this.data.offset.theta,
            .x = offset.x + @cos(offset.theta) * add_offset.x + @sin(offset.theta) * add_offset.y,
            .y = offset.y + -@sin(offset.theta) * add_offset.x + @cos(offset.theta) * add_offset.y,
        };
        this.offset = new_offset;
        for (this.children.items) |c| {
            c.forward(new_offset);
        }
    }
};

pub const JointTree = struct {
    root: *JointNode,
};

pub const Object = struct {
    root: Coords,
    children: std.ArrayList(Part),

    pub fn to_rects(this: *@This(), alloc: Allocator) RectList {
        var first_node = alloc.create(JointNode) catch {
            unreachable;
        };
        var first_joint = alloc.create(Joint) catch {
            unreachable;
        };
        first_joint.child = Object{
            .root = Coords{ .theta = 0.0, .x = 0.0, .y = 0.0 },
            .children = std.ArrayList(Part).init(alloc),
        };
        first_joint.offset = Coords{ .theta = 0.0, .x = 0.0, .y = 0.0 };
        first_node.data = first_joint;
        first_node.children = std.ArrayList(*JointNode).init(alloc);
        first_node.allocator = alloc;

        var vec = RectList{
            .root = this.root,
            .rects = std.ArrayList(Rect).init(alloc),
            .constraint_tree = JointTree{ .root = first_node },
        };
        this.to_rects_int(&vec, first_node);
        return vec;
    }

    fn to_rects_int(this: *@This(), vec: *RectList, constraints: *JointNode) void {
        var l = this.children.items.len;
        var i: usize = 0;
        while (i < l) : (i += 1) {
            var child = this.children.items[i];
            switch (child) {
                .rect => |rect| {
                    var new_rect = Rect{
                        .constraints = constraints,
                        .length = rect.length,
                        .width = rect.width,
                        // Do not compute the position here, wait for RectList.update
                        .pos = Coords{
                            .theta = 0.0,
                            .x = 0.0,
                            .y = 0.0,
                        },
                    };
                    vec.rects.append(new_rect) catch {};
                },
                .joint => |*joint| {
                    // No worry if not initialized now
                    var child_constraints = constraints.push(joint);
                    joint.child.to_rects_int(vec, child_constraints);
                },
            }
        }
    }
};

pub const RootObject = struct {
    static: bool,
    object: Object,
};

pub const RectList = struct {
    root: Coords,
    rects: std.ArrayList(Rect),
    constraint_tree: JointTree,

    pub fn update(this: *@This()) void {
        // Update all global joints position
        this.constraint_tree.root.forward(this.root);

        // Update all global rectangles position
        for (this.rects.items) |*rect| {
            var temp_pos = this.root;
            var prop_offset = rect.constraints.offset;
            temp_pos.x = @cos(prop_offset.theta) * rect.pos.x + @sin(prop_offset.theta) * rect.pos.y + prop_offset.x;
            temp_pos.y = -@sin(prop_offset.theta) * rect.pos.x + @cos(prop_offset.theta) * rect.pos.y + prop_offset.y;
            temp_pos.theta += prop_offset.theta;
            temp_pos.theta = @mod(temp_pos.theta, std.math.pi);
            rect.pos = temp_pos;
        }
    }

    // Very simple collision detection mechanism
    // Used to determine whether the
    pub fn collides(this: *@This()) bool {
        var l: usize = this.rects.items.length;
        var i: usize = 0;
        var j: usize = 0;

        while (i < l) : (i += 1) {
            j = 0;
            while (j < l) : (j += 1) {
                if (i == j) {
                    // Obviously do not consider this case
                    break;
                }
                var r1 = &this.rects.items[i];
                var r2 = &this.rects.items[j];
                if (r1.collides(r2)) {
                    return true;
                }
            }
        }
        return false;
    }
};

pub const Scene = struct {
    static: RectList,
    mobile: RectList,

    pub fn update(this: *@This()) void {
        this.static.update();
        this.mobile.update();
    }

    fn collides_inter(this: *@This()) bool {
        var l_i: usize = this.static.rects.items.length;
        var l_j: usize = this.mobile.rects.items.length;
        var i: usize = 0;
        var j: usize = 0;

        while (i < l_i) : (i += 1) {
            j = 0;
            while (j < l_j) : (j += 1) {
                var r1 = &this.static.rects.items[i];
                var r2 = &this.mobile.rects.items[j];
                if (r1.collides(r2)) {
                    return true;
                }
            }
        }
        return false;
    }

    pub fn collides(this: *@This()) bool {
        return this.static.collides() or this.mobile.collides() or this.collides_inter();
    }
};
