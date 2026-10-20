
// =========================================================================
// constants
// =========================================================================
const FX = 13.5;            // frame half length (X)
const FY = 14.0;            // frame half width (Y)
const RZ0 = 1.75;           // 2x1 frame tube bottom (SDS MK5n standard)
const RZ1 = 3.75;           // 2x1 frame tube top
const TW = 0.0625;          // tube wall (2x1x1/16, 1x1x1/16)
const MODX = 10.875;        // swerve steer axes (2.625 in from the frame faces)
const MODY = 11.375;
const MKLONG = 7.48;        // MK5n housing length along X
const MKFOOT = 6.25;        // MK5n footprint along Y
const MKTOP = 6.17;         // MK5n housing top
const MKBOT = 3.52;         // MK5n housing underside
const BPZ = 1.625;          // bellypan underside (1/8 6061 under the frame)

const ROTX = 0.7;           // serializer rotor axis (X)
const ROTP = 5.2;           // pocket circle radius (5 pockets)
const ROTR = 8.0;           // rotor gear pitch radius (160T 10DP)
const ROTZ = 4.5;           // rotor gear plate underside
const RFZ = 4.875;          // rotor floor top (FUEL rests here, centre 7.83)
const FINZ = 6.4;           // fin tops (pass under the uptake)
const BOWLR = 8.3;          // bowl wall inner radius
const BOWLPOSTS = [45, 150, 210, 315];   // bowl post angles about the rotor axis (clear of the cross members)
const BOWLZ0 = 6.0;         // bowl wall bottom / top
const BOWLZ1 = 9.0;
const TURX = -4.5;          // turret axis = rear (180 deg) pocket (ROTX - ROTP)
const UPB = 2.75;           // uptake contact faces at TURX -/+ UPB (5.5 in gap, 0.2 in squeeze)
const UPP = 3.75;           // uptake pulley centre offset (30T HTD 5 mm, pitch r 0.94)
const UPZ0 = 7.5;           // uptake lower / upper shaft heights
const UPZ1 = 11.0;
const UPY = 1.73;           // uptake tube centres |Y|
const DECKZ = 12.75;        // fixed turret deck underside (1/4 plate 12.75 - 13.0)
const DECKT = 13.0;         // deck top
// electrical layout (world inches)
const BATX = -10.95;        // battery centre (standing, long axis along Y, terminals toward the rear)
const BATY = 0.95;
const BATZ = 3.875;         // battery bottom (sled on top of the rear rail)
const BATGAP0 = -2.95;      // rear bumper gap for battery removal (R401 single gap > 1.25 in)
const BATGAP1 = 4.85;
const BRKX = -11.9;         // main breaker centre (deck rear pocket, under the cable-wrap ring)
const RSLX = -11.7;         // RSL outside the cable-wrap ring
const RSLY = -7.6;
const TRAYZ = 7.0;          // electronics board top
const RIOX = -8.95;
const RIOY = -10.75;
const PDHX = -7.53;
const PDHY = 10.75;
const TGROMX = -9.014;      // turret harness grommet through the deck, outside the wrap ring at WRAPAS
const TGROMY = 8.245;
const PIGX = -2.5;
const PIGY = 3.0;
const BEAMBOLTS = [-11.5, -9.0, -4.85, 4.85, 9.0, 11.5];   // deck-to-beam bolts at x -1.0 (clear of fence wing + bearing housing)
const BRGZ0 = 13.125;       // turret bearing (Kaydon KC090XP0, 9.0 x 10.0 x 0.5) bottom (housing on the deck)
const BRGZ1 = 13.625;       // turret bearing top = turret plate underside
const TPZ1 = 13.875;        // turret plate top
const FLYZ = 16.05;         // flywheel axis height
const FLYX = 4.35;          // flywheel axis offset from the turret axis (turret frame)
const HOODR = 7.3;          // hood inner radius about the flywheel axis
const FENR = 8.96;          // turret fence inner radius = wrap ring wall outer radius
const DECKR = 9.1;          // turret deck front arc radius
// gears meshing with the 120T turret plate sit on tooth gaps of the plate (angle about the turret axis)
const YAWA = 214.5;         // yaw pinion 12T
const ENC19A = 145.5;       // vernier CANcoder 19T
const ENC21A = 253.5;       // vernier CANcoder 21T
// cable wrap (service loop): fixed ring wall + rotating drum, loop in a horizontal U.
// Inner run centreline 7.33 (on the drum), outer run 8.73 (on the wall): the U-turn moves at
// k = 7.33 / 16.06 = 0.4564 of the turret angle. Travel +/-190 deg with 10 deg margin at each
// end: U0 = A + 10 + 190k, B0 = U0 + 190k - 200, length = 8.73 (U0 - A) + pi 0.70 + 7.33 (U0 - B0).
const WRAPA = 120.0;        // fixed end A (world deg)
const WRAPAS = 118.7;       // A-side harness entry angle (clear of the loop end)
const WRAPB0 = 103.44;      // moving end B (turret-local deg)
const WRAPBS = 102.14;      // B-side harness exit angle
const WRAPLEN = 31.43;      // loop length (in)
const WRAPRI = 7.33;        // inner run centreline radius
const WRAPRO = 8.73;        // outer run centreline radius
const WRAPZ = 14.305;       // loop centre height (loop 0.6 tall, 14.005 - 14.605)
const DRUMR0 = 7.1;         // drum wall (rotating) 7.10 - 7.16, lip out to 7.5 at 13.92 - 14.0
const DRUMR1 = 7.16;
const RINGRI = 8.55;        // ring floor (fixed) 8.55 - FENR at 13.85 - 14.0, wall 8.9 - FENR up to 14.7
const RINGRW = 8.9;

// intake (pivoting over-bumper)
const IOX = 12.75;
const IOZ = 10.21;
const ITH = 37 * degree;
const IL = 10.5;
const IYW = 11.5;
const IYP0 = 12.55;
const IYP1 = 12.8;

// hardware
const D10 = 0.201;          // #10 clearance (#7 drill)
const D14 = 0.257;          // 1/4 clearance
const BRG_HEX = 1.125;      // 1/2 in hex flanged bearing OD (WCP/Thrifty)
const BRG_HEX38 = 0.875;    // 3/8 hex flanged bearing OD (R6)

// =========================================================================
// appearance / materials
// =========================================================================
const C_AL = color(0.79, 0.80, 0.82);           // raw / clear anodised 6061
const C_ALBK = color(0.17, 0.17, 0.19);         // black anodised
const C_ALRD = color(0.70, 0.12, 0.12);         // red anodised (accent)
const C_ST = color(0.56, 0.57, 0.59);           // steel
const C_PCX = color(0.80, 0.89, 0.97, 0.28);    // clear polycarbonate
const C_PCS = color(0.15, 0.15, 0.17, 0.80);    // smoked polycarbonate
const C_BLK = color(0.07, 0.07, 0.08);
const C_SIL = color(0.62, 0.10, 0.10);          // red silicone / grip tape
const C_URE = color(0.88, 0.52, 0.10);          // orange urethane
const C_PETG = color(0.13, 0.13, 0.14);         // black printed PETG / Nylon
const C_UHMW = color(0.93, 0.93, 0.92);
const C_CF = color(0.11, 0.11, 0.12);           // carbon fibre
const C_WOOD = color(0.74, 0.58, 0.38);
const C_BUMPB = color(0.10, 0.25, 0.78);        // blue bumper fabric
const C_BUMPR = color(0.78, 0.10, 0.10);        // red bumper fabric
const C_WHT = color(0.95, 0.95, 0.95);
const C_RED = color(0.85, 0.08, 0.08);
const C_YEL = color(0.95, 0.80, 0.10);
const C_GRN = color(0.15, 0.62, 0.22);
const C_ORG = color(0.95, 0.45, 0.05);
const C_ETH = color(0.15, 0.45, 0.95);
const C_BRASS = color(0.78, 0.62, 0.25);
const C_GOLD = color(0.80, 0.62, 0.22);         // gold anodised
const C_BLUE = color(0.12, 0.30, 0.80);         // blue anodised
const C_TPU = color(0.20, 0.20, 0.22);          // black TPU / rubber

// densities g/cc
const DENS = {
    "Aluminum 6061" : 2.70, "Aluminum 7075" : 2.81, "Polycarbonate" : 1.20, "Steel" : 7.85,
    "UHMW" : 0.94, "Plywood" : 0.55, "Foam" : 0.04, "Rubber" : 1.10, "Nylon" : 1.15,
    "Carbon fiber" : 1.55, "PETG (40% infill)" : 0.55, "Urethane" : 1.15, "Silicone" : 1.10,
    "Cable" : 1.60, "Fabric" : 0.40, "COTS" : 1.0
};

function mat(name is string)
{
    return material(name, DENS[name] * gram / centimeter ^ 3);
}

function P(x is number, y is number, z is number) returns Vector
{
    return vector(x, y, z) * inch;
}

function vin(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * inch;
}

// =========================================================================
// part bookkeeping: every part is built under its own id (pid); solids are
// united, tools made under hid are subtracted, then name / colour / material.
// =========================================================================
function nextId(id is Id, c is box) returns Id
{
    c[] = c[] + 1;
    return id + ("n" ~ c[]);
}

function done(context is Context, pid is Id, hid is Id, name is string, col is Color, m is string)
{
    doneMass(context, pid, hid, name, col, m, -1);
}

function doneMass(context is Context, pid is Id, hid is Id, name is string, col is Color, m is string, lb is number)
{
    const bodies = qCreatedBy(pid, EntityType.BODY);
    if (size(evaluateQuery(context, bodies)) > 1)
        opBoolean(context, hid + "uni", { "tools" : bodies, "operationType" : BooleanOperationType.UNION });
    const holes = qCreatedBy(hid, EntityType.BODY);
    if (size(evaluateQuery(context, holes)) > 0)
        opBoolean(context, hid + "sub", {
                    "tools" : holes,
                    "targets" : qCreatedBy(pid, EntityType.BODY),
                    "operationType" : BooleanOperationType.SUBTRACTION
                });
    const q = qCreatedBy(pid, EntityType.BODY);
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : col });
    var mm = mat(m);
    if (lb > 0)
    {
        const vol = evVolume(context, { "entities" : q });
        mm = material(m ~ " (catalogue mass)", lb * 0.45359237 * kilogram / vol);
    }
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.MATERIAL, "value" : mm });
}

// =========================================================================
// coordinate systems and primitive solids
// =========================================================================
// (u, v) = (x, y), extrude +Z from z
function csXY(z is number) returns CoordSystem
{
    return coordSystem(P(0, 0, z), vector(1, 0, 0), vector(0, 0, 1));
}

// (u, v) = (x, z), extrude toward -Y starting at y
function csXZ(y is number) returns CoordSystem
{
    return coordSystem(P(0, y, 0), vector(1, 0, 0), vector(0, -1, 0));
}

// (u, v) = (y, z), extrude +X from x
function csYZ(x is number) returns CoordSystem
{
    return coordSystem(P(x, 0, 0), vector(0, 1, 0), vector(1, 0, 0));
}

// coordinate system with its Z along an arbitrary axis (a -> b)
function csAxis(a is Vector, axis is Vector) returns CoordSystem
{
    const z = normalize(axis);
    var x = cross(vector(0, 0, 1), z);
    if (norm(x) < 1e-6)
        x = vector(1, 0, 0);
    return coordSystem(a, normalize(x), z);
}

function W(cs is CoordSystem, u is number, v is number, w is number) returns Vector
{
    return toWorld(cs, P(u, v, w));
}

// closed polygon (unitless [u, v] points, inches) drawn into a sketch with every corner
// rounded by r using true tangent arcs (r is clamped per corner to fit the adjacent edges)
function skRoundPoly(sk is Sketch, key is string, pts is array, r is number)
{
    // path mode: entries [u, v, "A"] are arc mid-points between their neighbours
    var hasArc = false;
    for (var p in pts)
        if (size(p) > 2)
            hasArc = true;
    if (hasArc)
    {
        var i0 = 0;
        while (size(pts[i0]) > 2)
            i0 += 1;
        var q = [];
        for (var j = 0; j < size(pts); j += 1)
            q = append(q, pts[(i0 + j) % size(pts)]);
        q = append(q, pts[i0]);
        var i = 0;
        var s = 0;
        while (i < size(q) - 1)
        {
            s += 1;
            const a = vector(q[i][0], q[i][1]);
            if (size(q[i + 1]) > 2)
            {
                skArc(sk, key ~ "A" ~ s, { "start" : a * inch, "mid" : vector(q[i + 1][0], q[i + 1][1]) * inch, "end" : vector(q[i + 2][0], q[i + 2][1]) * inch });
                i += 2;
            }
            else
            {
                const b = vector(q[i + 1][0], q[i + 1][1]);
                if (norm(b - a) > 1e-5)
                    skLineSegment(sk, key ~ "L" ~ s, { "start" : a * inch, "end" : b * inch });
                i += 1;
            }
        }
        return;
    }
    const n = size(pts);
    var arcs = [];
    for (var i = 0; i < n; i += 1)
    {
        const V = vector(pts[i][0], pts[i][1]);
        const A = vector(pts[(i + n - 1) % n][0], pts[(i + n - 1) % n][1]);
        const B = vector(pts[(i + 1) % n][0], pts[(i + 1) % n][1]);
        const la = norm(A - V);
        const lb = norm(B - V);
        var rec = [V];
        if (r > 0 && la > 1e-6 && lb > 1e-6)
        {
            const a = (A - V) / la;
            const b = (B - V) / lb;
            const c = max(-1, min(1, dot(a, b)));
            if (c > -0.9995 && c < 0.9995)
            {
                const th = acos(c) / 2;
                var t = r / tan(th);
                var rr = r;
                const tmax = 0.45 * min(la, lb);
                if (t > tmax)
                {
                    t = tmax;
                    rr = t * tan(th);
                }
                const m = normalize(a + b);
                const C = V + m * (rr / sin(th));
                rec = [V + a * t, C - m * rr, V + b * t];
            }
        }
        arcs = append(arcs, rec);
    }
    for (var i = 0; i < n; i += 1)
    {
        const cur = arcs[i];
        const nxt = arcs[(i + 1) % n];
        if (size(cur) == 3)
            skArc(sk, key ~ "a" ~ i, { "start" : cur[0] * inch, "mid" : cur[1] * inch, "end" : cur[2] * inch });
        const s = size(cur) == 3 ? cur[2] : cur[0];
        const e = nxt[0];
        if (norm(e - s) > 1e-5)
            skLineSegment(sk, key ~ "l" ~ i, { "start" : s * inch, "end" : e * inch });
    }
}

// closed 2-D polygon in cs, corners rounded by r, extruded along cs Z from w0 to w1
function prismR(context is Context, sid is Id, cs is CoordSystem, pts is array, r is number, w0 is number, w1 is number)
{
    const pl = plane(W(cs, 0, 0, w0), cs.zAxis, cs.xAxis);
    const sk = newSketchOnPlane(context, sid + "sk", { "sketchPlane" : pl });
    skRoundPoly(sk, "p", pts, r);
    skSolve(sk);
    opExtrude(context, sid + "ex", {
                "entities" : qSketchRegion(sid + "sk"),
                "direction" : cs.zAxis,
                "endBound" : BoundingType.BLIND,
                "endDepth" : (w1 - w0) * inch
            });
    opDeleteBodies(context, sid + "del", { "entities" : qCreatedBy(sid + "sk", EntityType.BODY) });
}

// closed 2-D polygon in cs, extruded along cs Z from w0 to w1
function prism(context is Context, sid is Id, cs is CoordSystem, pts is array, w0 is number, w1 is number)
{
    prismR(context, sid, cs, pts, 0, w0, w1);
}

// fillet the edges of a just-built prism that run along its extrude direction
function filletAlong(context is Context, fid is Id, src is Id, dir is Vector, r is number)
{
    if (r <= 0)
        return;
    try silent
    {
        opFillet(context, fid, { "entities" : qParallelEdges(qCreatedBy(src, EntityType.EDGE), dir), "radius" : r * inch });
    }
}

function sBox(context is Context, pid is Id, c is box, a is Vector, b is Vector)
{
    fCuboid(context, nextId(pid, c), { "corner1" : a, "corner2" : b });
}

function sCyl(context is Context, pid is Id, c is box, a is Vector, b is Vector, r is number)
{
    fCylinder(context, nextId(pid, c), { "bottomCenter" : a, "topCenter" : b, "radius" : r * inch });
}

function sCone(context is Context, pid is Id, c is box, a is Vector, b is Vector, r0 is number, r1 is number)
{
    fCone(context, nextId(pid, c), { "bottomCenter" : a, "topCenter" : b, "bottomRadius" : r0 * inch, "topRadius" : r1 * inch });
}

function sTube(context is Context, pid is Id, hid is Id, c is box, a is Vector, b is Vector, ro is number, ri is number)
{
    sCyl(context, pid, c, a, b, ro);
    if (ri > 0)
    {
        const d = normalize(b - a) * 0.01 * inch;
        sCyl(context, hid, c, a - d, b + d, ri);
    }
}

function sPoly(context is Context, pid is Id, c is box, cs is CoordSystem, pts is array, w0 is number, w1 is number)
{
    prism(context, nextId(pid, c), cs, pts, w0, w1);
}

// regular hexagon (across flats af) centred at (u, v)
function hexPts(u is number, v is number, af is number) returns array
{
    const r = af / sqrt(3);
    var pts = [];
    for (var i = 0; i < 6; i += 1)
    {
        const a = (60 * i + 30) * degree;
        pts = append(pts, [u + r * cos(a), v + r * sin(a)]);
    }
    return pts;
}

function circPts(u is number, v is number, r is number, n is number) returns array
{
    var pts = [];
    for (var i = 0; i < n; i += 1)
    {
        const a = (360 * i / n) * degree;
        pts = append(pts, [u + r * cos(a), v + r * sin(a)]);
    }
    return pts;
}

// rectangle polygon; corners are rounded where it is used (plate "r" / "rp"), r kept for readability
function rrect(u0 is number, v0 is number, u1 is number, v1 is number, r is number) returns array
{
    return [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
}

// =========================================================================
// 2-D polygon utilities for pocketing
// =========================================================================
function polyArea(pts is array) returns number
{
    var a = 0;
    const n = size(pts);
    for (var i = 0; i < n; i += 1)
    {
        const p = pts[i];
        const q = pts[(i + 1) % n];
        a += p[0] * q[1] - q[0] * p[1];
    }
    return a / 2;
}

function ccw(pts is array) returns array
{
    if (polyArea(pts) >= 0)
        return pts;
    var r = [];
    for (var i = size(pts) - 1; i >= 0; i -= 1)
        r = append(r, pts[i]);
    return r;
}

// inset a convex polygon by d (edges moved inward); returns [] if it collapses
function polyInset(ptsIn is array, d is number) returns array
{
    const pts = ccw(ptsIn);
    const n = size(pts);
    var lines = [];
    for (var i = 0; i < n; i += 1)
    {
        const a = pts[i];
        const b = pts[(i + 1) % n];
        const dx = b[0] - a[0];
        const dy = b[1] - a[1];
        const L = sqrt(dx * dx + dy * dy);
        if (L < 1e-6)
            return [];
        lines = append(lines, [a[0] - dy / L * d, a[1] + dx / L * d, dx, dy]);
    }
    var out = [];
    for (var i = 0; i < n; i += 1)
    {
        const l1 = lines[(i + n - 1) % n];
        const l2 = lines[i];
        const det = l1[2] * l2[3] - l1[3] * l2[2];
        if (abs(det) < 1e-9)
            return [];
        const t = ((l2[0] - l1[0]) * l2[3] - (l2[1] - l1[1]) * l2[2]) / det;
        out = append(out, [l1[0] + t * l1[2], l1[1] + t * l1[3]]);
    }
    if (polyArea(out) <= 0.05 || polyArea(out) > polyArea(pts))
        return [];
    return out;
}

// Warren truss pockets in the rectangle [u0,u1]x[v0,v1] (outer border already removed),
// n bays, web width w. Returns inset triangles.
function trussPockets(u0 is number, v0 is number, u1 is number, v1 is number, n is number, w is number) returns array
{
    var tris = [];
    const L = (u1 - u0) / n;
    // bottom nodes at u0 + i L, top nodes at u0 + (i + 0.5) L
    tris = append(tris, [[u0, v0], [u0 + 0.5 * L, v1], [u0, v1]]);
    for (var i = 0; i < n; i += 1)
    {
        const b0 = u0 + i * L;
        tris = append(tris, [[b0, v0], [b0 + L, v0], [b0 + 0.5 * L, v1]]);
        if (i < n - 1)
            tris = append(tris, [[b0 + L, v0], [b0 + 1.5 * L, v1], [b0 + 0.5 * L, v1]]);
    }
    tris = append(tris, [[u1, v0], [u1, v1], [u1 - 0.5 * L, v1]]);
    var out = [];
    for (var t in tris)
    {
        const p = polyInset(t, w / 2);
        if (size(p) > 0)
            out = append(out, p);
    }
    return out;
}

// spoke pockets around a hub inside a convex boundary polygon: one pocket per
// boundary edge, trimmed by the hub boss radius rb. Web width w.
function fanPockets(boundary is array, hu is number, hv is number, rb is number, w is number) returns array
{
    const b = ccw(boundary);
    const n = size(b);
    var out = [];
    for (var i = 0; i < n; i += 1)
    {
        const p = b[i];
        const q = b[(i + 1) % n];
        const dp = sqrt((p[0] - hu) ^ 2 + (p[1] - hv) ^ 2);
        const dq = sqrt((q[0] - hu) ^ 2 + (q[1] - hv) ^ 2);
        if (dp < rb + 0.4 || dq < rb + 0.4)
            continue;
        const quad = [[hu + (p[0] - hu) * rb / dp, hv + (p[1] - hv) * rb / dp], p, q,
                [hu + (q[0] - hu) * rb / dq, hv + (q[1] - hv) * rb / dq]];
        const ins = polyInset(quad, w / 2);
        if (size(ins) > 0)
            out = append(out, ins);
    }
    return out;
}

// =========================================================================
// machined plate: outline in cs, thickness t (along cs Z from 0), with
// filleted outer corners, #10 / custom round holes, hex bores, bearing bores
// and filleted pockets.
// o: "r" outer fillet, "holes" [[u,v,d]], "hex" [[u,v,af]], "pockets" [[[u,v]...]],
//    "rp" pocket fillet radius, "w0" start offset along Z
// =========================================================================
function plate(context is Context, id is Id, k is string, name is string, cs is CoordSystem, outline is array, t is number, o is map, col is Color, m is string)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const w0 = o.w0 == undefined ? 0 : o.w0;
    prismR(context, pid + "o", cs, outline, o.r == undefined ? 0.125 : o.r, w0, w0 + t);
    const nh = o.holes == undefined ? 0 : size(o.holes);
    const nx = o.hex == undefined ? 0 : size(o.hex);
    const np = o.pockets == undefined ? 0 : size(o.pockets);
    if (nh + nx + np > 0)
    {
        const sk = newSketchOnPlane(context, hid + "sk", { "sketchPlane" : plane(W(cs, 0, 0, w0 - 0.05), cs.zAxis, cs.xAxis) });
        for (var i = 0; i < nh; i += 1)
            skCircle(sk, "c" ~ i, { "center" : vector(o.holes[i][0], o.holes[i][1]) * inch, "radius" : o.holes[i][2] / 2 * inch });
        for (var i = 0; i < nx; i += 1)
            skRoundPoly(sk, "x" ~ i ~ "_", hexPts(o.hex[i][0], o.hex[i][1], o.hex[i][2] + 0.01), 0.02);      // broached hex: 0.005 per flat clearance
        const rp = o.rp == undefined ? 0.125 : o.rp;
        for (var i = 0; i < np; i += 1)
            skRoundPoly(sk, "k" ~ i ~ "_", o.pockets[i], rp);
        skSolve(sk);
        opExtrude(context, hid + "ex", {
                    "entities" : qSketchRegion(hid + "sk"),
                    "direction" : cs.zAxis,
                    "endBound" : BoundingType.BLIND,
                    "endDepth" : (t + 0.1) * inch
                });
        opDeleteBodies(context, hid + "del", { "entities" : qCreatedBy(hid + "sk", EntityType.BODY) });
    }
    done(context, pid, hid, name, col, m);
}

// holes along a line (u0,v0)->(u1,v1) at pitch p
function holeRow(u0 is number, v0 is number, u1 is number, v1 is number, p is number, d is number) returns array
{
    const L = sqrt((u1 - u0) ^ 2 + (v1 - v0) ^ 2);
    const n = floor(L / p + 1e-6);
    const s0 = (L - n * p) / 2;          // centre the pattern on the line
    var out = [];
    for (var i = 0; i <= n; i += 1)
    {
        const s = (s0 + i * p) / max(L, 1e-9);
        out = append(out, [u0 + (u1 - u0) * s, v0 + (v1 - v0) * s, d]);
    }
    return out;
}

function concat(a is array, b is array) returns array
{
    var r = a;
    for (var x in b)
        r = append(r, x);
    return r;
}

// =========================================================================
// structural tube (box section) between a and b along a principal axis,
// with #10 holes at 0.5 in pitch through both face pairs.
// =========================================================================
function tube(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, wall is number, holes is boolean, col is Color)
{
    tube2(context, id, k, name, a, b, wall, holes, [], col);
}

// tube with extra bores: list of [point (Vector), axis dir (Vector), diameter]
function tube2(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, wall is number, holes is boolean, bores is array, col is Color)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    sBox(context, pid, c, a, b);
    const ai = [a[0] / inch, a[1] / inch, a[2] / inch];
    const bi = [b[0] / inch, b[1] / inch, b[2] / inch];
    const lo = [min(ai[0], bi[0]), min(ai[1], bi[1]), min(ai[2], bi[2])];
    const hi = [max(ai[0], bi[0]), max(ai[1], bi[1]), max(ai[2], bi[2])];
    const L = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    var ax = 0;
    for (var i = 1; i < 3; i += 1)
        if (L[i] > L[ax])
            ax = i;
    // hollow
    var ilo = [lo[0] + wall, lo[1] + wall, lo[2] + wall];
    var ihi = [hi[0] - wall, hi[1] - wall, hi[2] - wall];
    ilo[ax] = lo[ax] - 0.1;
    ihi[ax] = hi[ax] + 0.1;
    sBox(context, hid, c, vin(ilo), vin(ihi));
    if (holes)
    {
        const n = floor((L[ax] - 0.5) / 0.5 + 1e-6);
        const mid = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2];
        const s0 = lo[ax] + (L[ax] - n * 0.5) / 2 - mid[ax];
        var ea = [0, 0, 0];
        ea[ax] = 1;
        for (var j = 0; j < 3; j += 1)
        {
            if (j == ax)
                continue;
            var ej = [0, 0, 0];
            ej[j] = 1;
            var o = mid;
            o[j] = lo[j] - 0.05;
            const sid = hid + ("f" ~ j);
            const sk = newSketchOnPlane(context, sid + "sk", { "sketchPlane" : plane(vin(o), vector(ej[0], ej[1], ej[2]), vector(ea[0], ea[1], ea[2])) });
            for (var i = 0; i <= n; i += 1)
                skCircle(sk, "c" ~ i, { "center" : vector(s0 + i * 0.5, 0) * inch, "radius" : D10 / 2 * inch });
            skSolve(sk);
            opExtrude(context, sid + "ex", {
                        "entities" : qSketchRegion(sid + "sk"),
                        "direction" : vector(ej[0], ej[1], ej[2]),
                        "endBound" : BoundingType.BLIND,
                        "endDepth" : (L[j] + 0.1) * inch
                    });
            opDeleteBodies(context, sid + "del", { "entities" : qCreatedBy(sid + "sk", EntityType.BODY) });
        }
    }
    for (var bo in bores)
    {
        const d = normalize(bo[1]);
        sCyl(context, hid, c, bo[0] - d * 1.2 * inch, bo[0] + d * 1.2 * inch, bo[2] / 2);
    }
    done(context, pid, hid, name, col, "Aluminum 6061");
}

// =========================================================================
// power transmission and hardware
// =========================================================================
// hex shaft a -> b
function hexShaft(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, af is number, col is Color, m is string)
{
    const pid = id + k;
    const c = new box(0);
    const cs = csAxis(a, b - a);
    sPoly(context, pid, c, cs, hexPts(0, 0, af), 0, norm(b - a) / inch);
    done(context, pid, pid + "h", name, col, m);
    nodrill(context, qCreatedBy(pid, EntityType.BODY));
}

// tag bodies the fastener auto-drill must never cut (vendor parts, bearings, gears, shafts,
// belts, cables, fasteners): a bolt that runs into one of these stays a visible clash
function nodrill(context is Context, q is Query)
{
    setAttribute(context, { "entities" : q, "name" : "apexNoDrill", "attribute" : true });
}

// flanged hex-bore ball bearing centred at p, axis dir (flange on the +dir side)
function bearing(context is Context, id is Id, k is string, p is Vector, dir is Vector, od is number, af is number)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    const d = normalize(dir);
    const w = od > 1.0 ? 0.3125 : 0.28;
    sCyl(context, pid, c, p - d * (w / 2) * inch, p + d * (w / 2) * inch, od / 2);
    sCyl(context, pid, c, p + d * (w / 2 - 0.06) * inch, p + d * (w / 2) * inch, od / 2 + 0.08);
    const cs = csAxis(p - d * (w / 2 + 0.05) * inch, d);
    sPoly(context, hid, c, cs, hexPts(0, 0, af), 0, w + 0.1);
    done(context, pid, hid, (od > 1.0 ? "1/2 in hex flanged bearing (1.125 OD)" : "3/8 hex flanged bearing (0.875 OD)"), C_ST, "Steel");
    nodrill(context, qCreatedBy(pid, EntityType.BODY));
}

// round spacer / standoff (aluminium) between a and b
function spacer(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, ro is number, ri is number, col is Color)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    sTube(context, pid, hid, c, a, b, ro, ri);
    done(context, pid, hid, name, col, "Aluminum 6061");
}

// timing pulley (HTD 5 mm), pitch diameter from tooth count, with flanges and hex bore
function pulley(context is Context, id is Id, k is string, name is string, p is Vector, dir is Vector, teeth is number, width is number, af is number, col is Color)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    const d = normalize(dir);
    const rp = teeth * 5 / PI / 2 / 25.4;
    const cs = csAxis(p - d * (width / 2) * inch, d);
    // toothed rim (teeth as small notches)
    var pts = [];
    for (var i = 0; i < teeth; i += 1)
    {
        const a0 = 360 * i / teeth;
        const a1 = 360 * (i + 0.55) / teeth;
        pts = append(pts, [(rp - 0.03) * cos(a0 * degree), (rp - 0.03) * sin(a0 * degree)]);
        pts = append(pts, [(rp - 0.03) * cos(a1 * degree), (rp - 0.03) * sin(a1 * degree)]);
        pts = append(pts, [(rp - 0.09) * cos((a1 + 0.1 * 360 / teeth) * degree), (rp - 0.09) * sin((a1 + 0.1 * 360 / teeth) * degree)]);
        pts = append(pts, [(rp - 0.09) * cos((a0 + 0.9 * 360 / teeth) * degree), (rp - 0.09) * sin((a0 + 0.9 * 360 / teeth) * degree)]);
    }
    sPoly(context, pid, c, cs, pts, 0, width);
    // flanges
    sCyl(context, pid, c, W(cs, 0, 0, -0.04), W(cs, 0, 0, 0), rp + 0.08);
    sCyl(context, pid, c, W(cs, 0, 0, width), W(cs, 0, 0, width + 0.04), rp + 0.08);
    sPoly(context, hid, c, cs, hexPts(0, 0, af), -0.1, width + 0.1);
    done(context, pid, hid, name, col, "Aluminum 6061");
    nodrill(context, qCreatedBy(pid, EntityType.BODY));
}

// timing belt loop around two pulleys whose centres are a and b (pitch radii ra, rb),
// belt plane normal n, width w (centred on the a-b line)
function beltLoop(context is Context, id is Id, k is string, name is string, a is Vector, ra is number, b is Vector, rb is number, n is Vector, w is number)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    const nn = normalize(n);
    const ex = normalize(b - a);
    const cs = coordSystem(a - nn * (w / 2) * inch, ex, nn);
    const L = norm(b - a) / inch;
    sPoly(context, pid, c, cs, hull2(0, ra + 0.11, L, rb + 0.11), 0, w);
    sPoly(context, hid, c, cs, hull2(0, ra + 0.02, L, rb + 0.02), -0.1, w + 0.1);
    done(context, pid, hid, name, C_BLK, "Rubber");
    nodrill(context, qCreatedBy(pid, EntityType.BODY));
}

// outline of the convex hull of two circles on the u axis (centres 0 and L)
function hull2(u0 is number, r0 is number, L is number, r1 is number) returns array
{
    const s = (r0 - r1) / L;
    const ph = acos(max(-1, min(1, s)));
    var pts = [];
    const n = 14;
    // arc around circle 1 (at L) from -ph to +ph
    for (var i = 0; i <= n; i += 1)
    {
        const a = -ph + 2 * ph * i / n;
        pts = append(pts, [L + r1 * cos(a), r1 * sin(a)]);
    }
    // arc around circle 0 from ph to 2pi - ph
    for (var i = 0; i <= n; i += 1)
    {
        const a = ph + (2 * PI * radian - 2 * ph) * i / n;
        pts = append(pts, [u0 + r0 * cos(a), r0 * sin(a)]);
    }
    return pts;
}

// spur gear (trapezoid teeth) in cs, centred at (u, v), with hub bore (hex af or round rb)
function gear(context is Context, id is Id, k is string, name is string, cs is CoordSystem, u is number, v is number, teeth is number, dp is number, t is number, af is number, col is Color, m is string)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    const rp = teeth / dp / 2;
    const ra = rp + 1 / dp;
    const rr = rp - 1.25 / dp;
    const pa = 2 * PI / teeth;
    var pts = [];
    for (var i = 0; i < teeth; i += 1)
    {
        const a = i * pa;
        pts = append(pts, [u + rr * cos((a - 0.30 * pa) * radian), v + rr * sin((a - 0.30 * pa) * radian)]);
        pts = append(pts, [u + ra * cos((a - 0.12 * pa) * radian), v + ra * sin((a - 0.12 * pa) * radian)]);
        pts = append(pts, [u + ra * cos((a + 0.12 * pa) * radian), v + ra * sin((a + 0.12 * pa) * radian)]);
        pts = append(pts, [u + rr * cos((a + 0.30 * pa) * radian), v + rr * sin((a + 0.30 * pa) * radian)]);
    }
    sPoly(context, pid, c, cs, pts, 0, t);
    if (af > 0)
        sPoly(context, hid, c, cs, hexPts(u, v, af), -0.1, t + 0.1);
    done(context, pid, hid, name, col, m);
    nodrill(context, qCreatedBy(pid, EntityType.BODY));
}

// #10-32 cap screws: list of [head point, shank direction, length, (nut), (flat head)].
// Button heads sit on the surface at the head point; flat heads (82 deg) are flush with it.
// The shank is modelled at the thread minor diameter (0.156) so it checks clean in tapped
// holes and vendor-model tapped holes. One template per (length, nut, head) group is patterned
// onto every location; each bolt stays its own part so the BOM counts them.
// Clearance holes: every part in the drill scope that a shank actually passes through without
// a hole gets a #10 clearance hole along the bolt (and a countersink for a flat head). Parts
// tagged apexNoDrill (vendor models, bearings, gears, pulleys, belts, shafts, cables, other
// fasteners) are never cut, so a bolt in the wrong place still shows up as a clash.
function bolts(context is Context, id is Id, k is string, list is array)
{
    boltsIn(context, id, k, list, qCreatedBy(id, EntityType.BODY));
}

function boltsIn(context is Context, id is Id, k is string, list is array, scope is Query)
{
    if (size(list) == 0)
        return;
    var groups = {};
    for (var b in list)
    {
        const key = "L" ~ round(b[2] * 1000) ~ ((size(b) > 3 && b[3]) ? "n" : "p") ~ ((size(b) > 4 && b[4]) ? "f" : "b");
        groups[key] = append(groups[key] == undefined ? [] : groups[key], b);
    }
    var g = 0;
    var made = [];
    for (var e in groups)
    {
        g += 1;
        const blist = e.value;
        const L = blist[0][2];
        const nut = size(blist[0]) > 3 && blist[0][3];
        const flat = size(blist[0]) > 4 && blist[0][4];
        const tid = id + (k ~ "t" ~ g);
        const c = new box(0);
        if (flat)
        {
            fCone(context, nextId(tid, c), { "bottomCenter" : P(0, 0, -0.002), "topCenter" : P(0, 0, 0.1), "bottomRadius" : 0.19 * inch, "topRadius" : 0.0785 * inch });
            fCylinder(context, nextId(tid, c), { "bottomCenter" : P(0, 0, 0.09), "topCenter" : P(0, 0, L), "radius" : 0.078 * inch });
        }
        else
        {
            fCylinder(context, nextId(tid, c), { "bottomCenter" : P(0, 0, -0.04), "topCenter" : P(0, 0, -0.002), "radius" : 0.19 * inch });
            fCone(context, nextId(tid, c), { "bottomCenter" : P(0, 0, -0.04), "topCenter" : P(0, 0, -0.105), "bottomRadius" : 0.19 * inch, "topRadius" : 0.12 * inch });
            fCylinder(context, nextId(tid, c), { "bottomCenter" : P(0, 0, -0.02), "topCenter" : P(0, 0, L), "radius" : 0.078 * inch });
        }
        if (nut)
            prism(context, nextId(tid, c), csXY(0), hexPts(0, 0, 0.375), L - 0.25, L - 0.06);
        opBoolean(context, tid + "u", { "tools" : qCreatedBy(tid, EntityType.BODY), "operationType" : BooleanOperationType.UNION });
        var T = [];
        var names = [];
        var i = 0;
        for (var b in blist)
        {
            i += 1;
            T = append(T, toWorld(csAxis(b[0], b[1])));
            names = append(names, "b" ~ i);
        }
        const pt = id + (k ~ "p" ~ g);
        opPattern(context, pt, { "entities" : qCreatedBy(tid, EntityType.BODY), "transforms" : T, "instanceNames" : names });
        opDeleteBodies(context, id + (k ~ "d" ~ g), { "entities" : qCreatedBy(tid, EntityType.BODY) });
        const q = qCreatedBy(pt, EntityType.BODY);
        setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME,
                    "value" : "#10-32 x " ~ toString(roundTo(L, 0.125)) ~ (flat ? " FHCS" : " BHCS") ~ (nut ? " + nylock" : "") });
        setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : C_BLK });
        setProperty(context, { "entities" : q, "propertyType" : PropertyType.MATERIAL, "value" : mat("Steel") });
        made = append(made, q);
    }
    const bq = qUnion(made);
    nodrill(context, bq);

    // ---- clearance holes where a shank passes through a drillable part
    const targets = qSubtraction(scope, qUnion([bq, qHasAttribute("apexNoDrill")]));
    if (size(evaluateQuery(context, targets)) == 0)
        return;
    const cl = evCollision(context, { "tools" : bq, "targets" : targets });
    var hits = {};            // target body transient id -> [target query, [bolt indices]]
    for (var x in cl)
    {
        if (x["type"] != ClashType.INTERFERE && x["type"] != ClashType.TOOL_IN_TARGET && x["type"] != ClashType.TARGET_IN_TOOL)
            continue;
        // which bolt of the list is this body: nearest shank midpoint
        const bb = evBox3d(context, { "topology" : x.toolBody, "tight" : true });
        const mid = (bb.minCorner + bb.maxCorner) / 2;
        var best = -1;
        var bd = 1e9 * inch;
        for (var j = 0; j < size(list); j += 1)
        {
            const d = normalize(list[j][1]);
            const m2 = list[j][0] + d * (list[j][2] / 2) * inch;
            const dd = norm(m2 - mid);
            if (dd < bd)
            {
                bd = dd;
                best = j;
            }
        }
        const key = toString(x.targetBody);
        if (hits[key] == undefined)
            hits[key] = [x.targetBody, []];
        hits[key][1] = append(hits[key][1], best);
    }
    var t = 0;
    for (var e in hits)
    {
        t += 1;
        const did = id + (k ~ "dr" ~ t);
        const c = new box(0);
        for (var j in e.value[1])
        {
            const b = list[j];
            const d = normalize(b[1]);
            const flat = size(b) > 4 && b[4];
            fCylinder(context, nextId(did, c), { "bottomCenter" : b[0] - d * 0.02 * inch, "topCenter" : b[0] + d * (b[2] + 0.02) * inch, "radius" : D10 / 2 * inch });
            if (flat)
                fCone(context, nextId(did, c), { "bottomCenter" : b[0] - d * 0.02 * inch, "topCenter" : b[0] + d * 0.105 * inch, "bottomRadius" : 0.21 * inch, "topRadius" : D10 / 2 * inch });
        }
        opBoolean(context, did + "sub", { "tools" : qCreatedBy(did, EntityType.BODY), "targets" : e.value[0], "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

function roundTo(x is number, s is number) returns number
{
    return round(x / s) * s;
}

// =========================================================================
// purchased parts
// =========================================================================
function cots(context is Context, id is Id, build is function, T is Transform, name is string, lb is number)
{
    const inst = newInstantiator(id);
    addInstance(inst, build, { "transform" : T });
    instantiate(context, inst);
    const q = qCreatedBy(id, EntityType.BODY);
    nodrill(context, q);
    if (lb > 0)
    {
        const vol = evVolume(context, { "entities" : q });
        setProperty(context, { "entities" : q, "propertyType" : PropertyType.MATERIAL, "value" : material(name ~ " (catalogue mass)", lb * 0.45359237 * kilogram / vol) });
    }
}

// transform from a local frame (origin o, local x -> ex, local z -> ez) into world
function frameT(o is Vector, ex is Vector, ez is Vector) returns Transform
{
    return toWorld(coordSystem(o, normalize(ex), normalize(ez)));
}

// Kraken motors (MKCad models, local Y = motor axis). Placed by the centre of the
// mounting face and the shaft direction; body (2.94 in) extends behind the face,
// the shaft 1.47 in beyond it. X44 model origin is the face; X60 origin is 1.23 in behind it.
function kraken(context is Context, id is Id, face is Vector, shaft is Vector, isX60 is boolean)
{
    const s = normalize(shaft);
    var ex = cross(s, vector(0, 0, 1));
    if (norm(ex) < 1e-6)
        ex = vector(1, 0, 0);
    ex = normalize(ex);
    const ez = cross(ex, -s);
    if (isX60)
        cots(context, id, KX60::build, frameT(face - s * 1.23 * inch, ex, ez), "Kraken X60", 1.20);
    else
        cots(context, id, KX44::build, frameT(face, ex, ez), "Kraken X44", 0.75);
}

// =========================================================================
// solids of revolution and turned parts
// =========================================================================
// closed profile of [a, r] points (a along axis from o, r >= 0 radial), revolved 360 deg
function revolveSolid(context is Context, sid is Id, o is Vector, axis is Vector, pts is array)
{
    const z = normalize(axis);
    var x = cross(z, vector(0, 0, 1));
    if (norm(x) < 1e-6)
        x = vector(1, 0, 0);
    x = normalize(x);
    const sk = newSketchOnPlane(context, sid + "sk", { "sketchPlane" : plane(o, x, z) });
    var p2 = [];
    for (var p in pts)
        p2 = append(p2, vector(p[0], p[1]) * inch);
    p2 = append(p2, vector(pts[0][0], pts[0][1]) * inch);
    skPolyline(sk, "poly", { "points" : p2 });
    skSolve(sk);
    opRevolve(context, sid + "rv", { "entities" : qSketchRegion(sid + "sk"), "axis" : line(o, z), "angleForward" : 360 * degree });
    opDeleteBodies(context, sid + "del", { "entities" : qCreatedBy(sid + "sk", EntityType.BODY) });
}

// turned part: revolve profile, optional hex bore (af > 0) or round bore (rb > 0)
function turned(context is Context, id is Id, k is string, name is string, o is Vector, axis is Vector, pts is array, af is number, rb is number, col is Color, m is string)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    revolveSolid(context, nextId(pid, c), o, axis, pts);
    var amin = 1e9;
    var amax = -1e9;
    for (var p in pts)
    {
        amin = min(amin, p[0]);
        amax = max(amax, p[0]);
    }
    const d = normalize(axis);
    if (af > 0)
        sPoly(context, hid, c, csAxis(o + d * (amin - 0.05) * inch, d), hexPts(0, 0, af), 0, amax - amin + 0.1);
    if (rb > 0)
        sCyl(context, hid, c, o + d * (amin - 0.05) * inch, o + d * (amax + 0.05) * inch, rb);
    done(context, pid, hid, name, col, m);
}

// flanged ball bearing: bore hex (af > 0) or round (rb > 0); flange on the +dir face
function fbearing(context is Context, id is Id, k is string, p is Vector, dir is Vector, od is number, w is number, af is number, rb is number)
{
    const fl = od / 2 + 0.07;
    const ri = (af > 0 ? af * 0.62 : rb + 0.12);
    const h = w / 2;
    turned(context, id, k, (af > 0 ? toString(af) ~ " hex" : toString(rb * 2) ~ " round") ~ " bore flanged bearing (" ~ toString(od) ~ " OD)",
        p, dir, [[-h, ri], [-h, od / 2 - 0.03], [-h + 0.03, od / 2], [h - 0.06, od / 2], [h - 0.06, fl], [h, fl], [h, ri]], af, rb, C_ST, "Steel");
    nodrill(context, qCreatedBy(id + k, EntityType.BODY));
}

// shaft collar (clamping, 1/2 hex or round), width w
function collar(context is Context, id is Id, k is string, p is Vector, dir is Vector, od is number, w is number, af is number, rb is number)
{
    turned(context, id, k, "Shaft collar", p, dir, [[-w / 2, 0.05], [-w / 2, od / 2 - 0.03], [-w / 2 + 0.03, od / 2], [w / 2 - 0.03, od / 2], [w / 2, od / 2 - 0.03], [w / 2, 0.05]], af, rb, C_BLK, "Aluminum 6061");
    nodrill(context, qCreatedBy(id + k, EntityType.BODY));
}

// wheel: urethane/rubber tread on a hub, recessed sides
function wheel(context is Context, id is Id, k is string, name is string, p is Vector, dir is Vector, dia is number, w is number, af is number, col is Color, m is string)
{
    const R = dia / 2;
    const h = w / 2;
    turned(context, id, k, name, p, dir,
        [[-h, 0.2], [-h, 0.85], [-h + 0.18, 0.95], [-h + 0.18, R - 0.32], [-h, R - 0.22], [-h, R - 0.1], [-h + 0.1, R], [h - 0.1, R], [h, R - 0.1], [h, R - 0.22],
                [h - 0.18, R - 0.32], [h - 0.18, 0.95], [h, 0.85], [h, 0.2]], af, 0, col, m);
    nodrill(context, qCreatedBy(id + k, EntityType.BODY));
}

// hex standoff (#10, 3/8 hex) from a to b, tapped (shown as 0.16 bore)
function standoff(context is Context, id is Id, k is string, a is Vector, b is Vector)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    const L = norm(b - a) / inch;
    const cs = csAxis(a, b - a);
    sPoly(context, pid, c, cs, hexPts(0, 0, 0.375), 0, L);
    sCyl(context, hid, c, a - normalize(b - a) * 0.05 * inch, b + normalize(b - a) * 0.05 * inch, 0.08);
    done(context, pid, hid, "#10-32 x " ~ toString(roundTo(L, 0.125)) ~ " hex standoff", C_AL, "Aluminum 6061");
}

// spur gear plate: tooth outline (pitch circle teeth / dp) + plate cutouts (holes / hex / pockets in o)
function gearPts(u is number, v is number, teeth is number, dp is number) returns array
{
    return gearPtsR(u, v, teeth, dp, 0);
}

// tooth outline with tooth 0 centred at rot degrees (sketch angle)
function gearPtsR(u is number, v is number, teeth is number, dp is number, rot is number) returns array
{
    const rp = teeth / dp / 2;
    const ra = rp + 1 / dp;
    const rr = rp - 1.25 / dp;
    const pa = 2 * PI / teeth;
    var pts = [];
    for (var i = 0; i < teeth; i += 1)
    {
        const a = i * pa + rot / 180 * PI;
        pts = append(pts, [u + rr * cos((a - 0.30 * pa) * radian), v + rr * sin((a - 0.30 * pa) * radian)]);
        pts = append(pts, [u + ra * cos((a - 0.12 * pa) * radian), v + ra * sin((a - 0.12 * pa) * radian)]);
        pts = append(pts, [u + ra * cos((a + 0.12 * pa) * radian), v + ra * sin((a + 0.12 * pa) * radian)]);
        pts = append(pts, [u + rr * cos((a + 0.30 * pa) * radian), v + rr * sin((a + 0.30 * pa) * radian)]);
    }
    return pts;
}

// HTD 5 mm timing pulley with flanges (revolved hub + toothed ring), hex bore
function htd(context is Context, id is Id, k is string, p is Vector, dir is Vector, teeth is number, bw is number, af is number)
{
    pulley(context, id, k, toString(teeth) ~ "T HTD 5 mm pulley (" ~ toString(bw) ~ " in belt)", p, dir, teeth, bw + 0.06, af, C_AL);
}

// arc of points about (cu, cv), radius r, angles a0 -> a1 (degrees), n segments
function arcPts(cu is number, cv is number, r is number, a0 is number, a1 is number, n is number) returns array
{
    var pts = [];
    for (var i = 0; i <= n; i += 1)
    {
        const a = (a0 + (a1 - a0) * i / n) * degree;
        pts = append(pts, [cu + r * cos(a), cv + r * sin(a)]);
    }
    return pts;
}

// true circular arc for skRoundPoly paths: from the point at angle a0 (already in the
// path) to a1 about (cu, cv), split into pieces of at most 120 deg
function arcTo(cu is number, cv is number, r is number, a0 is number, a1 is number) returns array
{
    const n = max(1, ceil(abs(a1 - a0) / 120 - 1e-9));
    var out = [];
    for (var i = 0; i < n; i += 1)
    {
        const am = a0 + (a1 - a0) * (i + 0.5) / n;
        const ae = a0 + (a1 - a0) * (i + 1) / n;
        out = append(out, [cu + r * cos(am * degree), cv + r * sin(am * degree), "A"]);
        out = append(out, [cu + r * cos(ae * degree), cv + r * sin(ae * degree)]);
    }
    return out;
}

// full circle as a path
function circPath(cu is number, cv is number, r is number) returns array
{
    return concat([[cu + r, cv]], arcTo(cu, cv, r, 0, 360));
}

// reverse an array
function rev(a is array) returns array
{
    var r = [];
    for (var i = size(a) - 1; i >= 0; i -= 1)
        r = append(r, a[i]);
    return r;
}

// rotate every body created under rid about a vertical axis through (cx, cy)
function spinZ(context is Context, id is Id, q is Query, cx is number, cy is number, ang is ValueWithUnits)
{
    if (abs(ang) < 1e-9 * degree)
        return;
    opTransform(context, id, { "bodies" : q, "transform" : rotationAround(line(P(cx, cy, 0), vector(0, 0, 1)), ang) });
}

// rotate bodies about an arbitrary axis line through p along d
function spinAxis(context is Context, id is Id, q is Query, p is Vector, d is Vector, ang is ValueWithUnits)
{
    if (abs(ang) < 1e-9 * degree)
        return;
    opTransform(context, id, { "bodies" : q, "transform" : rotationAround(line(p, normalize(d)), ang) });
}

// bolts from other subsystems that pass through the bellypan (the bellypan gets a hole at each)
function bellyBolts() returns array
{
    var k = [[ROTX - 0.5, 0], [ROTX + 0.5, 0]];         // rotor axle block
    for (var a in [60, 180, 300])
        k = append(k, [ROTX + 7.3 * cos(a * degree), 7.3 * sin(a * degree)]);     // rotor thrust pucks
    for (var a in BOWLPOSTS)
        k = append(k, [ROTX + 8.7 * cos(a * degree), 8.7 * sin(a * degree)]);     // bowl posts
    for (var q in [[-1.2, -9.75], [1.2, -9.75]])                                   // rotor drive pod legs
        k = append(k, [ROTX + q[0], q[1]]);
    return concat(k, elecBellyPts());
}

// posts (deck -> wrap ring floor), r 8.72 about the turret axis, clear of the deck electronics
function ringPosts() returns array
{
    var p = [];
    for (var a in [30, 90, 150, 210, 270, 330])
        p = append(p, [TURX + 8.72 * cos(a * degree), 8.72 * sin(a * degree)]);
    return p;
}

// bellypan keep-outs: mounting points of other subsystems that need solid bellypan
function bellyKeep() returns array
{
    return concat(concat([[ROTX, 0]], bellyBolts()), elecBellyKeep());
}

// =========================================================================
// automatic truss pocketing: an isometric triangle lattice clipped to the
// plate outline (inset by `border`) and kept clear of bosses [u, v, r]
// =========================================================================
function pip(p is array, poly is array) returns boolean
{
    var c = false;
    const n = size(poly);
    for (var i = 0; i < n; i += 1)
    {
        const a = poly[i];
        const b = poly[(i + 1) % n];
        if (((a[1] > p[1]) != (b[1] > p[1])) && (p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]))
            c = !c;
    }
    return c;
}

function segDist(p is array, a is array, b is array) returns number
{
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    const L2 = dx * dx + dy * dy;
    var t = L2 < 1e-12 ? 0 : ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2;
    t = max(0, min(1, t));
    const qx = a[0] + t * dx - p[0];
    const qy = a[1] + t * dy - p[1];
    return sqrt(qx * qx + qy * qy);
}

function polyDist(p is array, poly is array) returns number
{
    var d = 1e9;
    const n = size(poly);
    for (var i = 0; i < n; i += 1)
        d = min(d, segDist(p, poly[i], poly[(i + 1) % n]));
    return d;
}

// outline must be a plain point polygon (arcs sampled)
function autoTruss(outline is array, bosses is array, cell is number, web is number, border is number) returns array
{
    var u0 = 1e9;
    var v0 = 1e9;
    var u1 = -1e9;
    var v1 = -1e9;
    for (var p in outline)
    {
        u0 = min(u0, p[0]);
        v0 = min(v0, p[1]);
        u1 = max(u1, p[0]);
        v1 = max(v1, p[1]);
    }
    const h = cell * 0.8660254;
    var out = [];
    for (var j = 0; v0 + j * h < v1; j += 1)
        for (var i = -1; u0 + i * cell < u1; i += 1)
        {
            const off = (j % 2 == 0) ? 0 : cell / 2;
            const x0 = u0 + i * cell + off;
            const y0 = v0 + j * h;
            for (var t in [[[x0, y0], [x0 + cell, y0], [x0 + cell / 2, y0 + h]],
                    [[x0 + cell / 2, y0 + h], [x0 + cell, y0], [x0 + cell * 1.5, y0 + h]]])
            {
                const ins = polyInset(t, web / 2);
                if (size(ins) == 0)
                    continue;
                var ok = true;
                for (var p in ins)
                    if (!pip(p, outline) || polyDist(p, outline) < border)
                        ok = false;
                if (ok)
                    for (var p in outline)
                        if (pip(p, ins))
                            ok = false;
                if (ok)
                    for (var b in bosses)
                        if (pip([b[0], b[1]], ins) || polyDist([b[0], b[1]], ins) < b[2])
                            ok = false;
                if (ok)
                    out = append(out, ins);
            }
        }
    return out;
}

// bosses from a hole list [[u, v, d]]: keep (d / 2 + margin) solid around each
function holeBosses(holes is array, margin is number) returns array
{
    var b = [];
    for (var h in holes)
        b = append(b, [h[0], h[1], h[2] / 2 + margin]);
    return b;
}

// partial solid of revolution about a vertical axis from angle a0 to a1 (deg); profile [z, r]
function revolveArcZ(context is Context, sid is Id, cx is number, cy is number, pts is array, a0 is number, a1 is number)
{
    const u = vector(cos(a0 * degree), sin(a0 * degree), 0);
    const nrm = vector(-sin(a0 * degree), cos(a0 * degree), 0);
    const o = P(cx, cy, 0);
    const sk = newSketchOnPlane(context, sid + "sk", { "sketchPlane" : plane(o, nrm, vector(0, 0, 1)) });
    var p2 = [];
    for (var p in pts)
        p2 = append(p2, vector(p[0], p[1]) * inch);
    p2 = append(p2, vector(pts[0][0], pts[0][1]) * inch);
    skPolyline(sk, "poly", { "points" : p2 });
    skSolve(sk);
    opRevolve(context, sid + "rv", { "entities" : qSketchRegion(sid + "sk"), "axis" : line(o, vector(0, 0, 1)), "angleForward" : (a1 - a0) * degree });
    opDeleteBodies(context, sid + "del", { "entities" : qCreatedBy(sid + "sk", EntityType.BODY) });
}
