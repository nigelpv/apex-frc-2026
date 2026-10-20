FeatureScript 3095;
import(path : "onshape/std/common.fs", version : "3095.0");

/* =====================================================================
 * APEX - 2026 REBUILT competition robot, parametric model library.
 * Robot frame: origin at frame centre on the carpet, +X = intake (front),
 * +Y = left, +Z = up. All dimensions in inches.
 * Frame 27.0 (X) x 28.0 (Y) in = 110 in perimeter (R104). Max height 21.75 in
 * (TRENCH clearance 22.25 in). Turret axis at X = -3.5. Rotor centre X = 2.5.
 * ===================================================================== */

// ---------------------------------------------------------------- constants
const FX = 13.5;          // frame half length
const FY = 14.0;          // frame half width
const RZ0 = 1.5;          // frame rail bottom
const RZ1 = 3.5;          // frame rail top
const MODX = 10.875;      // swerve module centres
const MODY = 11.375;
const ROTX = 2.5;         // serializer rotor centre
const ROTR = 9.0;         // rotor radius (180T 10DP ring)
const ROTZ = 4.5;         // rotor disk bottom
const TURX = -3.5;        // turret axis (= rear pocket of the rotor)
const DECKZ = 12.75;      // fixed turret deck bottom
const FLYZ = 16.05;       // flywheel axis height
const FLYX = 4.35;        // flywheel axis offset from turret axis (turret-local)
const MAXH = 21.75;
const UPB = 2.75;         // uptake belt ball-side face offset from the turret axis (X)
const UPP = 3.47;         // uptake pulley centre offset (24T 5 mm HTD, r 0.72)
const UPZ0 = 8.0;         // uptake lower / upper pulley heights
const UPZ1 = 11.5;
const FENR = 9.55;        // turret fence radius (turret swing radius 9.36)
// intake (pivoting over-bumper): pivot O, ramp angle when deployed, pivot -> front roller
const IOX = 12.75;
const IOZ = 10.21;
const ITH = 37 * degree;
const IL = 10.73;
const IYW = 11.5;         // half width of rollers / ramp (23.0 in throat)
const IYP0 = 12.6;        // arm plates |Y| 12.6 - 12.85
const IYP1 = 12.85;

// colours
const C_AL = color(0.78, 0.79, 0.81);
const C_ALDK = color(0.35, 0.36, 0.38);
const C_PC = color(0.80, 0.90, 0.98);
const C_PCDK = color(0.30, 0.32, 0.36);
const C_ST = color(0.55, 0.56, 0.58);
const C_BLK = color(0.08, 0.08, 0.09);
const C_RED = color(0.85, 0.10, 0.10);
const C_BLUE = color(0.10, 0.30, 0.85);
const C_YEL = color(0.95, 0.80, 0.10);
const C_GRN = color(0.10, 0.60, 0.20);
const C_ORG = color(0.95, 0.45, 0.05);
const C_ETH = color(0.15, 0.45, 0.95);
const C_WHT = color(0.95, 0.95, 0.95);
const C_KRAKEN = color(0.12, 0.12, 0.14);
const C_SIL = color(0.85, 0.30, 0.20);
const C_WOOD = color(0.72, 0.55, 0.35);
const C_GOLD = color(0.85, 0.65, 0.15);
const C_CF = color(0.15, 0.15, 0.16);
const C_GRY = color(0.50, 0.50, 0.52);

// densities g/cc
const DENS = {
    "Aluminum 6061" : 2.70, "Aluminum 7075" : 2.81, "Polycarbonate" : 1.20, "Steel" : 7.85,
    "HDPE" : 0.95, "Plywood" : 0.55, "EVA foam" : 0.06, "Copper" : 3.2, "Rubber" : 1.10,
    "Nylon" : 1.15, "Carbon fiber" : 1.55, "Electronics" : 1.10, "Motor" : 3.30,
    "SRPP" : 0.92, "Battery" : 2.40, "TPU" : 1.20, "Silicone" : 1.10, "Cable" : 1.60,
    "EVA foam + TPU skin" : 0.12, "PETG print (40% infill)" : 0.55
};

// Purchased (COTS) parts are modelled as simplified envelopes, so their mass is
// set from the vendor catalogue instead of envelope volume x density. Keyed by
// the part-number prefix of the part name (lb). Kraken masses are set in motor().
// Sources: SDS MK5n page (3.9 lb module w/o motors + encoder), reca.lc / WCP docs
// (X60 1.20 lb, X44 0.75 lb); the rest are vendor datasheet values rounded up.
const CAT_LB = {
    "DT-11" : 0.6, "DT-12" : 0.6, "DT-13" : 0.6, "DT-14" : 0.6,      // MK5n top plate + steering gear
    "DT-21" : 2.5, "DT-22" : 2.5, "DT-23" : 2.5, "DT-24" : 2.5,      // MK5n housing, fork, bevels, bearings
    "DT-31" : 0.8, "DT-32" : 0.8, "DT-33" : 0.8, "DT-34" : 0.8,      // 4 in spike-grip wheel + hub
    "DT-61" : 0.05, "DT-62" : 0.05, "DT-63" : 0.05, "DT-64" : 0.05,  // CANcoder
    "HP-11" : 0.35, "HP-23" : 0.05, "HP-24" : 0.05, "HP-26" : 0.30,
    "IN-05" : 0.06, "IN-21" : 0.30,
    "TS-01" : 0.45, "TS-02" : 0.60, "TS-05" : 0.08, "TS-06" : 0.35, "TS-23" : 0.33,
    "EL-01" : 0.85, "EL-03" : 12.9, "EL-04" : 0.10, "EL-05" : 0.20, "EL-06" : 0.25, "EL-07" : 0.45,
    "EL-08" : 0.70, "EL-09" : 0.06, "EL-10" : 0.20, "EL-11" : 0.15, "EL-12" : 0.06, "EL-13" : 0.15,
    "EL-14" : 0.33, "EL-15" : 0.33, "EL-16" : 0.40
};

function P(x is number, y is number, z is number) returns Vector
{
    return vector(x, y, z) * inch;
}

function vin(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * inch;
}

// intake arm coordinates (deployed): s along the ramp from the pivot, n normal (up, away from the floor)
function aP(s is number, n is number) returns array
{
    return [IOX + s * cos(ITH) + n * sin(ITH), IOZ - s * sin(ITH) + n * cos(ITH)];
}

function vXZ(p is array, y is number) returns Vector
{
    return P(p[0], y, p[1]);
}

function mat(name is string)
{
    return material(name, DENS[name] * gram / centimeter ^ 3);
}

// ---------------------------------------------------------------- builder
// Every part is built under its own Id (pid). Solids are made under pid + "s#",
// united, holes (made under a separate id) subtracted, then named/coloured.

function nextId(id is Id, c is box) returns Id
{
    c[] = c[] + 1;
    return id + ("n" ~ c[]);
}

function sBox(context is Context, pid is Id, c is box, a is Vector, b is Vector)
{
    fCuboid(context, nextId(pid, c), { "corner1" : a, "corner2" : b });
}

function sCyl(context is Context, pid is Id, c is box, a is Vector, b is Vector, r is number)
{
    fCylinder(context, nextId(pid, c), { "bottomCenter" : a, "topCenter" : b, "radius" : r * inch });
}

function sSph(context is Context, pid is Id, c is box, a is Vector, r is number)
{
    fSphere(context, nextId(pid, c), { "center" : a, "radius" : r * inch });
}

// Prism from a closed 2-D polygon. plane: "XY" (extrude +Z from z0 to z1),
// "XZ" (polygon in X,Z, extrude +Y from y0 to y1), "YZ" (polygon in Y,Z, extrude +X)
function sPrism(context is Context, pid is Id, c is box, pln is string, pts is array, d0 is number, d1 is number)
{
    const sid = nextId(pid, c);
    var pl;
    var dir;
    if (pln == "XY")
    {
        pl = plane(P(0, 0, d0), vector(0, 0, 1), vector(1, 0, 0));
        dir = vector(0, 0, 1);
    }
    else if (pln == "XZ")
    {
        pl = plane(P(0, d0, 0), vector(0, -1, 0), vector(1, 0, 0));
        dir = vector(0, 1, 0);
    }
    else
    {
        pl = plane(P(d0, 0, 0), vector(1, 0, 0), vector(0, 1, 0));
        dir = vector(1, 0, 0);
    }
    const sk = newSketchOnPlane(context, sid + "sk", { "sketchPlane" : pl });
    var p2 = [];
    for (var p in pts)
        p2 = append(p2, vector(p[0], p[1]) * inch);
    p2 = append(p2, vector(pts[0][0], pts[0][1]) * inch);
    skPolyline(sk, "poly", { "points" : p2 });
    skSolve(sk);
    opExtrude(context, sid + "ex", {
                "entities" : qSketchRegion(sid + "sk"),
                "direction" : dir,
                "endBound" : BoundingType.BLIND,
                "endDepth" : (d1 - d0) * inch
            });
    opDeleteBodies(context, sid + "del", { "entities" : qCreatedBy(sid + "sk", EntityType.BODY) });
}

// Spur gear (simple trapezoidal teeth) in the XY plane about centre (cx, cy)
function sGear(context is Context, pid is Id, c is box, cx is number, cy is number, teeth is number, dp is number, z0 is number, z1 is number)
{
    const rp = teeth / dp / 2;
    const ra = rp + 1 / dp;
    const rr = rp - 1.25 / dp;
    const pa = 2 * PI / teeth;
    var pts = [];
    for (var i = 0; i < teeth; i += 1)
    {
        const a = i * pa;
        pts = append(pts, [cx + rr * cos((a - 0.30 * pa) * radian), cy + rr * sin((a - 0.30 * pa) * radian)]);
        pts = append(pts, [cx + ra * cos((a - 0.12 * pa) * radian), cy + ra * sin((a - 0.12 * pa) * radian)]);
        pts = append(pts, [cx + ra * cos((a + 0.12 * pa) * radian), cy + ra * sin((a + 0.12 * pa) * radian)]);
        pts = append(pts, [cx + rr * cos((a + 0.30 * pa) * radian), cy + rr * sin((a + 0.30 * pa) * radian)]);
    }
    sPrism(context, pid, c, "XY", pts, z0, z1);
}

function hBox(context is Context, hid is Id, c is box, a is Vector, b is Vector)
{
    fCuboid(context, nextId(hid, c), { "corner1" : a, "corner2" : b });
}

function hCyl(context is Context, hid is Id, c is box, a is Vector, b is Vector, r is number)
{
    fCylinder(context, nextId(hid, c), { "bottomCenter" : a, "topCenter" : b, "radius" : r * inch });
}

// finish a part: union solids, subtract holes, name, colour, material
function done(context is Context, pid is Id, hid is Id, name is string, col is Color, m is string)
{
    var lb = -1;
    const code = match(name, "^([A-Z][A-Z]-[0-9][0-9]).*");
    if (code.hasMatch && CAT_LB[code.captures[1]] != undefined)
        lb = CAT_LB[code.captures[1]];
    doneMass(context, pid, hid, name, col, m, lb);
}

// as done(), but lb > 0 forces the part mass (density = mass / modelled volume)
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

// convenience single-primitive parts
function partBox(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, col is Color, m is string)
{
    const pid = id + k;
    const c = new box(0);
    sBox(context, pid, c, a, b);
    done(context, pid, pid + "h", name, col, m);
}

function partCyl(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, r is number, col is Color, m is string)
{
    const pid = id + k;
    const c = new box(0);
    sCyl(context, pid, c, a, b, r);
    done(context, pid, pid + "h", name, col, m);
}

function partTube(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, ro is number, ri is number, col is Color, m is string)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    sCyl(context, pid, c, a, b, ro);
    const d = normalize(b - a) * 0.05 * inch;
    hCyl(context, hid, c, a - d, b + d, ri);
    done(context, pid, hid, name, col, m);
}

// Kraken X60 / X44 envelope: body + face plate + shaft along axis a->b
function motor(context is Context, id is Id, k is string, name is string, base is Vector, axis is Vector, isX60 is boolean)
{
    const pid = id + k;
    const c = new box(0);
    const L = isX60 ? 2.60 : 2.05;
    const ax = normalize(axis);
    const top = base + ax * L * inch;
    sCyl(context, pid, c, base, top, 1.18);
    sCyl(context, pid, c, top, top + ax * 0.55 * inch, 0.25);
    sCyl(context, pid, c, base - ax * 0.35 * inch, base, 0.95);
    doneMass(context, pid, pid + "h", name ~ (isX60 ? " (Kraken X60)" : " (Kraken X44)"), C_KRAKEN, "Motor", isX60 ? 1.20 : 0.75);
}

// rectangular tube along X, Y or Z with optional #10 hole pattern on the wide faces
function rectTube(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, wall is number, holeAxis is string)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    sBox(context, pid, c, a, b);
    const ai = [a[0] / inch, a[1] / inch, a[2] / inch];
    const bi = [b[0] / inch, b[1] / inch, b[2] / inch];
    const lx = abs(bi[0] - ai[0]);
    const ly = abs(bi[1] - ai[1]);
    const lz = abs(bi[2] - ai[2]);
    const x0 = min(ai[0], bi[0]);
    const y0 = min(ai[1], bi[1]);
    const z0 = min(ai[2], bi[2]);
    // hollow: shrink in the two non-long directions
    if (lx >= ly && lx >= lz)
    {
        hBox(context, hid, c, P(x0 - 0.1, y0 + wall, z0 + wall), P(x0 + lx + 0.1, y0 + ly - wall, z0 + lz - wall));
        if (holeAxis == "Y")
            for (var x = x0 + 0.5; x < x0 + lx - 0.4; x += 1.0)
                hCyl(context, hid, c, P(x, y0 - 0.1, z0 + lz / 2), P(x, y0 + ly + 0.1, z0 + lz / 2), 0.098);
    }
    else if (ly >= lx && ly >= lz)
    {
        hBox(context, hid, c, P(x0 + wall, y0 - 0.1, z0 + wall), P(x0 + lx - wall, y0 + ly + 0.1, z0 + lz - wall));
        if (holeAxis == "X")
            for (var y = y0 + 0.5; y < y0 + ly - 0.4; y += 1.0)
                hCyl(context, hid, c, P(x0 - 0.1, y, z0 + lz / 2), P(x0 + lx + 0.1, y, z0 + lz / 2), 0.098);
    }
    else
    {
        hBox(context, hid, c, P(x0 + wall, y0 + wall, z0 - 0.1), P(x0 + lx - wall, y0 + ly - wall, z0 + lz + 0.1));
    }
    done(context, pid, hid, name, C_AL, "Aluminum 6061");
}

// wire / cable: chain of cylinders through waypoints. Interior ends are
// extended by 0.9 r so consecutive segments overlap and union into one body.
function wire(context is Context, id is Id, k is string, name is string, pts is array, r is number, col is Color)
{
    const pid = id + k;
    const c = new box(0);
    const n = size(pts);
    for (var i = 0; i < n - 1; i += 1)
    {
        var a = vin(pts[i]);
        var b = vin(pts[i + 1]);
        const len = norm(b - a);
        if (len > 0.02 * inch)
        {
            const u = (b - a) / len;
            if (i > 0)
                a = a - u * (0.9 * r * inch);
            if (i < n - 2)
                b = b + u * (0.9 * r * inch);
            sCyl(context, pid, c, a, b, r);
        }
    }
    done(context, pid, pid + "h", name, col, "Cable");
}

function rotZ(context is Context, id is Id, q is Query, cx is number, cy is number, ang is ValueWithUnits)
{
    if (abs(ang / degree) > 1e-6)
        opTransform(context, id, { "bodies" : q, "transform" : rotationAround(line(P(cx, cy, 0), vector(0, 0, 1)), ang) });
}

function rotY(context is Context, id is Id, q is Query, cx is number, cz is number, ang is ValueWithUnits)
{
    if (abs(ang / degree) > 1e-6)
        opTransform(context, id, { "bodies" : q, "transform" : rotationAround(line(P(cx, 0, cz), vector(0, 1, 0)), ang) });
}

function moveBy(context is Context, id is Id, q is Query, dx is number, dy is number, dz is number)
{
    if (abs(dx) + abs(dy) + abs(dz) > 1e-9)
        opTransform(context, id, { "bodies" : q, "transform" : transform(P(dx, dy, dz)) });
}

/* =====================================================================
 * 01 DRIVETRAIN: 2x1x0.1 6061 frame, 0.09 in bellypan, 4x SDS MK5n (R1)
 * ===================================================================== */
annotation { "Feature Type Name" : "APEX 01 Drivetrain" }
export const apexDrivetrain = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Module steer angle" }
        isAngle(definition.steer, ANGLE_360_ZERO_DEFAULT_BOUNDS);
    }
    {
        // frame rails run between the MK5n corner blocks (module footprint 5.25 in square,
        // outer faces flush with the frame perimeter)
        const MH = 2.625;
        rectTube(context, id, "rF", "DT-01 Front rail 2x1x0.1 (17.5 in)", P(FX - 1, -(MODY - MH), RZ0), P(FX, MODY - MH, RZ1), 0.1, "X");
        rectTube(context, id, "rB", "DT-02 Rear rail 2x1x0.1 (17.5 in)", P(-FX, -(MODY - MH), RZ0), P(-FX + 1, MODY - MH, RZ1), 0.1, "X");
        rectTube(context, id, "rL", "DT-03 Left rail 2x1x0.1 (16.5 in)", P(-(MODX - MH), FY - 1, RZ0), P(MODX - MH, FY, RZ1), 0.1, "Y");
        rectTube(context, id, "rR", "DT-04 Right rail 2x1x0.1 (16.5 in)", P(-(MODX - MH), -FY, RZ0), P(MODX - MH, -FY + 1, RZ1), 0.1, "Y");
        // mid cross rail at the rotor/battery boundary (stiffens frame, carries battery tray)
        rectTube(context, id, "rM", "DT-05 Cross rail 2x1x0.1 (26.0 in)", P(-7.6, -FY + 1, RZ0), P(-6.6, FY - 1, RZ1), 0.1, "X");

        // bellypan with lightening pockets (0.09 6061, router)
        {
            const pid = id + "belly";
            const hid = id + "bellyh";
            const c = new box(0);
            sBox(context, pid, c, P(-FX + 1, -FY + 1, RZ0 - 0.09), P(FX - 1, FY - 1, RZ0));
            // module corner notches
            for (var sx in [-1, 1])
                for (var sy in [-1, 1])
                    hBox(context, hid, c, P(sx * MODX - 2.6, sy * MODY - 2.6, 0), P(sx * MODX + 2.6, sy * MODY + 2.6, 2));
            // pockets (keep 0.6 in webs)
            for (var x = -7.6; x < 7.0; x += 3.4)
                for (var y = -7.9; y < 7.5; y += 3.4)
                    if (!(abs(x - ROTX) < 1.8 && abs(y) < 1.8))
                        hBox(context, hid, c, P(x, y, 0), P(x + 2.8, y + 2.8, 2));
            done(context, pid, hid, "DT-06 Bellypan 0.090 6061 (pocketed)", C_ALDK, "Aluminum 6061");
        }

        // swerve modules (SDS MK5n envelope: top plate, fork, 4 in spike wheel, X60 drive, X44 steer, CANcoder)
        var n = 0;
        for (var sx in [1, -1])
            for (var sy in [1, -1])
            {
                n += 1;
                const mx = sx * MODX;
                const my = sy * MODY;
                const tg = (sx > 0 ? "F" : "B") ~ (sy > 0 ? "L" : "R");
                const mid = id + ("mod" ~ n);
                // top plate (sits on the rails, acts as the corner gusset)
                {
                    const pid = mid + "plate";
                    const hid = mid + "plateh";
                    const c = new box(0);
                    const p0 = [sx > 0 ? mx - 2.6 : -FX, sy > 0 ? my - 2.6 : -FY];
                    const p1 = [sx > 0 ? FX : mx + 2.6, sy > 0 ? FY : my + 2.6];
                    sBox(context, pid, c, P(p0[0], p0[1], RZ1), P(p1[0], p1[1], RZ1 + 0.25));
                    hCyl(context, hid, c, P(mx, my, RZ1 - 1), P(mx, my, RZ1 + 1), 1.9);
                    done(context, pid, hid, "DT-1" ~ n ~ " MK5n top plate " ~ tg, C_ALDK, "Aluminum 6061");
                }
                // rotating fork + wheel, rotated by steer angle
                {
                    const pid = mid + "fork";
                    const c = new box(0);
                    sCyl(context, pid, c, P(mx, my, RZ1 - 0.6), P(mx, my, RZ1), 1.85);
                    sBox(context, pid, c, P(mx - 1.9, my - 1.15, 0.9), P(mx + 0.4, my - 0.95, RZ1 - 0.6));
                    sBox(context, pid, c, P(mx - 1.9, my + 0.95, 0.9), P(mx + 0.4, my + 1.15, RZ1 - 0.6));
                    done(context, pid, pid + "h", "DT-2" ~ n ~ " MK5n fork + bevel housing " ~ tg, C_AL, "Aluminum 7075");
                    rotZ(context, mid + "rf", qCreatedBy(pid, EntityType.BODY), mx, my, definition.steer);
                }
                {
                    const pid = mid + "wheel";
                    const hid = mid + "wheelh";
                    const c = new box(0);
                    sCyl(context, pid, c, P(mx, my - 0.75, 2.0), P(mx, my + 0.75, 2.0), 2.0);
                    hCyl(context, hid, c, P(mx, my - 1, 2.0), P(mx, my + 1, 2.0), 0.6);
                    done(context, pid, hid, "DT-3" ~ n ~ " 4 in spike-grip wheel " ~ tg, C_BLK, "Rubber");
                    rotZ(context, mid + "rw", qCreatedBy(pid, EntityType.BODY), mx, my, definition.steer);
                }
                motor(context, mid, "drive", "DT-4" ~ n ~ " Drive motor " ~ tg, P(mx, my, RZ1 + 0.9), vector(0, 0, 1), true);
                // drive motor sits on the module centreline; steer motor offset inboard
                motor(context, mid, "steer", "DT-5" ~ n ~ " Steer motor " ~ tg, P(mx - sx * 2.55, my - sy * 0.9, RZ1 + 0.9), vector(0, 0, 1), false);
                partBox(context, mid, "cc", "DT-6" ~ n ~ " CANcoder " ~ tg, P(mx - 0.75, my + sy * 2.05 - 0.45, RZ1 + 0.25), P(mx + 0.75, my + sy * 2.05 + 0.45, RZ1 + 0.95), C_BLK, "Electronics");
            }
        // drive motors overlap the module top plate face: shift them so they read correctly
    });

/* =====================================================================
 * 02 BUMPERS: 3/4 in plywood backing, 2x 2.5 in pool-noodle padding,
 * 5.0 in tall (Z 1.75-6.75 fills BUMPER ZONE 2.5-5.75), 3.25 in thick,
 * rear split for the battery / main-breaker access gap (R401).
 * ===================================================================== */
annotation { "Feature Type Name" : "APEX 02 Bumpers" }
export const apexBumpers = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Red alliance" }
        definition.red is boolean;
    }
    {
        const col = definition.red ? C_RED : C_BLUE;
        const z0 = 1.75;
        const z1 = 6.75;
        const T = 3.25;
        const B = 0.75;
        // segments: [name, backing box, padding box]
        const segs = [
                ["BU-01 Front bumper", [FX, -FY - T, FX + B, FY + T], [FX + B, -FY - T, FX + T, FY + T]],
                ["BU-02 Left bumper", [-FX, FY, FX, FY + B], [-FX, FY + B, FX, FY + T]],
                ["BU-03 Right bumper", [-FX, -FY - B, FX, -FY], [-FX, -FY - T, FX, -FY - B]],
                ["BU-04 Rear-left bumper", [-FX - B, 4.0, -FX, FY + T], [-FX - T, 4.0, -FX - B, FY + T]],
                ["BU-05 Rear-right bumper", [-FX - B, -FY - T, -FX, -8.0], [-FX - T, -FY - T, -FX - B, -8.0]]
            ];
        var i = 0;
        for (var s in segs)
        {
            i += 1;
            const b = s[1];
            const p = s[2];
            partBox(context, id, "bk" ~ i, s[0] ~ " backing 3/4 plywood", P(b[0], b[1], z0), P(b[2], b[3], z1), C_WOOD, "Plywood");
            partBox(context, id, "pd" ~ i, s[0] ~ " padding + cover", P(p[0], p[1], z0), P(p[2], p[3], z1), col, "EVA foam");
        }
        // numbers: white placards (team number goes here, 4 in digits, 3 sides)
        partBox(context, id, "nF", "BU-10 Number placard front", P(FX + T, -4, 2.6), P(FX + T + 0.02, 4, 6.0), C_WHT, "Nylon");
        partBox(context, id, "nL", "BU-11 Number placard left", P(-4, FY + T, 2.6), P(4, FY + T + 0.02, 6.0), C_WHT, "Nylon");
        partBox(context, id, "nR", "BU-12 Number placard right", P(-4, -FY - T - 0.02, 2.6), P(4, -FY - T, 6.0), C_WHT, "Nylon");
    });

/* =====================================================================
 * 03 HOPPER + SERIALIZER: 1/16 polycarb walls, turret deck + fence, 18 in
 * 6-pocket rotor (180T 10DP ring) on X60 10T pinion (18:1), fixed exit lift
 * blade, uptake with twin 15 mm HTD belts (fore/aft of the ball) on X60.
 * ===================================================================== */
annotation { "Feature Type Name" : "APEX 03 Hopper + Serializer" }
export const apexHopper = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Rotor angle" }
        isAngle(definition.rotor, ANGLE_360_ZERO_DEFAULT_BOUNDS);
    }
    {
        // side walls 1/16 polycarb (sit on the module top plates and rails), 1/16 aluminium top angle
        for (var sy in [1, -1])
        {
            const k = sy > 0 ? "L" : "R";
            {
                const pid = id + ("wall" ~ k);
                const hid = id + ("wallh" ~ k);
                const c = new box(0);
                sBox(context, pid, c, P(-FX, sy * (FY - 0.0625), RZ1 + 0.25), P(FX, sy * FY, 21.0));
                // windows over the swerve modules for service
                hBox(context, hid, c, P(-FX + 1.5, sy * FY - 1, RZ1 + 0.75), P(-FX + 6.0, sy * FY + 1, 6.6));
                hBox(context, hid, c, P(FX - 6.0, sy * FY - 1, RZ1 + 0.75), P(FX - 1.5, sy * FY + 1, 6.6));
                // camera window for the side Limelight
                hBox(context, hid, c, P(-1.6, sy * FY - 1, 18.1), P(1.0, sy * FY + 1, 19.7));
                done(context, pid, hid, "HP-0" ~ (sy > 0 ? 1 : 2) ~ " Hopper side wall " ~ k ~ " 1/16 polycarb", C_PC, "Polycarbonate");
            }
            partBox(context, id, "ang" ~ k, "HP-0" ~ (sy > 0 ? 3 : 4) ~ " Wall top angle " ~ k ~ " 1x1x1/16",
                P(-FX, sy * (FY - 1.0), 21.0), P(FX, sy * FY, 21.0625), C_AL, "Aluminum 6061");
        }
        partBox(context, id, "wallB", "HP-05 Rear wall 1/16 polycarb", P(-FX, -FY + 0.0625, RZ1 + 0.25), P(-FX + 0.0625, FY - 0.0625, DECKZ + 0.125), C_PC, "Polycarbonate");

        // fixed turret deck 1/8 6061 (top face at DECKZ + 0.25, 1x1 angle stiffened), bearing bore 10.3 in, lightening
        {
            const pid = id + "deck";
            const hid = id + "deckh";
            const c = new box(0);
            sBox(context, pid, c, P(-FX + 0.0625, -FY + 0.0625, DECKZ + 0.125), P(4.4, FY - 0.0625, DECKZ + 0.25));
            hCyl(context, hid, c, P(TURX, 0, DECKZ - 1), P(TURX, 0, DECKZ + 1), 5.15);
            for (var sy in [1, -1])
            {
                hBox(context, hid, c, P(-11.8, sy * 7.0, DECKZ - 1), P(-6.2, sy * 12.4, DECKZ + 1));
                hBox(context, hid, c, P(-1.0, sy * 7.0, DECKZ - 1), P(3.2, sy * 12.4, DECKZ + 1));
            }
            hCyl(context, hid, c, P(TURX - 6.5 * cos(150 * degree), 6.5 * sin(150 * degree), DECKZ - 1), P(TURX - 6.5 * cos(150 * degree), 6.5 * sin(150 * degree), DECKZ + 1), 0.6);
            done(context, pid, hid, "HP-06 Turret deck 1/8 6061", C_AL, "Aluminum 6061");
        }

        // floor fill blocks: sloped TPU-skinned foam that funnels FUEL into the rotor bowl
        for (var sy in [1, -1])
        {
            const pid = id + ("ramp" ~ (sy > 0 ? "L" : "R"));
            const hid = id + ("ramph" ~ (sy > 0 ? "L" : "R"));
            const c = new box(0);
            const pts = sy > 0 ? [[5.0, RZ1 + 0.25], [12.6, RZ1 + 0.25], [12.6, 7.6], [5.0, RZ1 + 0.6]]
                               : [[-12.6, RZ1 + 0.25], [-5.0, RZ1 + 0.25], [-5.0, RZ1 + 0.6], [-12.6, 7.6]];
            sPrism(context, pid, c, "YZ", pts, -6.4, 12.4);
            hCyl(context, hid, c, P(ROTX, 0, 0), P(ROTX, 0, 10), ROTR + 0.45);
            for (var sx in [1, -1])
                hBox(context, hid, c, P(sx * MODX - 4.0, sy * MODY - 3.4, 0), P(sx * MODX + 4.0, sy * MODY + 3.4, 10));
            hCyl(context, hid, c, P(ROTX + 9.5 * cos(-40 * degree), 9.5 * sin(-40 * degree), 0), P(ROTX + 9.5 * cos(-40 * degree), 9.5 * sin(-40 * degree), 10), 1.8);
            done(context, pid, hid, "HP-0" ~ (sy > 0 ? 7 : 8) ~ " Floor fill block " ~ (sy > 0 ? "L" : "R") ~ " (EVA core, TPU skin, slopes to rotor)", C_PCDK, "EVA foam + TPU skin");
        }
        // under-deck closeouts: keep FUEL from rolling under the deck outside the bowl
        for (var sy in [1, -1])
        {
            const k = sy > 0 ? "L" : "R";
            const pid = id + ("clo" ~ k);
            const hid = id + ("cloh" ~ k);
            const c = new box(0);
            sBox(context, pid, c, P(-6.5, sy * 9.45, 7.6), P(4.4, sy * (FY - 0.0625), DECKZ + 0.1));
            hCyl(context, hid, c, P(ROTX, 0, 0), P(ROTX, 0, 20), ROTR + 0.45);
            if (sy > 0)
                hCyl(context, hid, c, P(TURX + UPP, 9.2, UPZ1), P(TURX + UPP, 13.5, UPZ1), 1.3);
            done(context, pid, hid, "HP-28 Under-deck closeout " ~ k ~ " (EVA foam)", C_PCDK, "EVA foam + TPU skin");
        }

        // rotor bowl wall (fixed); rear opening lets the uptake struts / back pulley through
        {
            const pid = id + "bowl";
            const hid = id + "bowlh";
            const c = new box(0);
            sCyl(context, pid, c, P(ROTX, 0, ROTZ + 0.55), P(ROTX, 0, 7.5), ROTR + 0.35);
            hCyl(context, hid, c, P(ROTX, 0, ROTZ - 1), P(ROTX, 0, 8.5), ROTR + 0.22);
            hBox(context, hid, c, P(TURX - 4.4, -1.0, ROTZ), P(TURX - 2.4, 1.0, 8.5));
            done(context, pid, hid, "HP-09 Rotor bowl ring 1/8 polycarb (rolled)", C_PC, "Polycarbonate");
        }
        // rotor: 180T 10DP 7075 ring gear (1.25 in wide) bolted to a 1/8 polycarb floor
        // disk, with 6 fins + centre cone printed in PETG (one rigid rotating assembly).
        // Fins are slotted at r 5.7 - 6.3 so the fixed exit blade passes through them.
        {
            const pid = id + "rotG";
            const hid = id + "rotGh";
            const c = new box(0);
            sGear(context, pid, c, ROTX, 0, 180, 10, ROTZ, ROTZ + 0.25);
            hCyl(context, hid, c, P(ROTX, 0, ROTZ - 1), P(ROTX, 0, ROTZ + 1), 7.75);
            done(context, pid, hid, "HP-10a Rotor ring gear 180T 10DP 1/4 7075", C_GOLD, "Aluminum 7075");
        }
        {
            const pid = id + "rotD";
            const hid = id + "rotDh";
            const c = new box(0);
            sCyl(context, pid, c, P(ROTX, 0, ROTZ + 0.125), P(ROTX, 0, ROTZ + 0.25), 7.75);
            hCyl(context, hid, c, P(ROTX, 0, ROTZ - 1), P(ROTX, 0, ROTZ + 1), 0.5);
            done(context, pid, hid, "HP-10b Rotor floor disk 1/8 polycarb", C_PC, "Polycarbonate");
        }
        {
            const pid = id + "rotor";
            const hid = id + "rotorh";
            const c = new box(0);
            for (var i = 0; i < 6; i += 1)
            {
                const a = i * 60 * degree + 30 * degree;
                for (var seg in [[2.6, 5.7], [6.3, 8.5]])
                    sPrism(context, pid, c, "XY", [
                                [ROTX + seg[0] * cos(a) + 0.125 * sin(a), seg[0] * sin(a) - 0.125 * cos(a)],
                                [ROTX + seg[1] * cos(a) + 0.125 * sin(a), seg[1] * sin(a) - 0.125 * cos(a)],
                                [ROTX + seg[1] * cos(a) - 0.125 * sin(a), seg[1] * sin(a) + 0.125 * cos(a)],
                                [ROTX + seg[0] * cos(a) - 0.125 * sin(a), seg[0] * sin(a) + 0.125 * cos(a)]
                            ], ROTZ + 0.25, 7.25);
            }
            fCone(context, nextId(pid, c), { "bottomCenter" : P(ROTX, 0, ROTZ + 0.25), "topCenter" : P(ROTX, 0, 8.4), "bottomRadius" : 2.7 * inch, "topRadius" : 0.6 * inch });
            hCyl(context, hid, c, P(ROTX, 0, ROTZ - 1), P(ROTX, 0, 9), 0.25);
            done(context, pid, hid, "HP-10c Rotor fins (6 slotted pockets) + centre cone, PETG print", C_GOLD, "PETG print (40% infill)");
            rotZ(context, id + "rotRot", qUnion([qCreatedBy(id + "rotG", EntityType.BODY), qCreatedBy(id + "rotD", EntityType.BODY),
                        qCreatedBy(pid, EntityType.BODY)]), ROTX, 0, definition.rotor);
        }
        partCyl(context, id, "rhub", "HP-11 Rotor dead axle 1/2 in + flanged bearing stack", P(ROTX, 0, RZ0), P(ROTX, 0, ROTZ), 0.5, C_ST, "Steel");
        // rotor drive: X60 standing on the bellypan under the ring gear, 10T 10DP pinion (18:1)
        {
            const a = -40 * degree;
            const r = 9.0 + 0.5;
            const px = ROTX + r * cos(a);
            const py = r * sin(a);
            {
                const pid = id + "pin";
                const c = new box(0);
                sGear(context, pid, c, px, py, 10, 10, ROTZ, ROTZ + 0.25);
                done(context, pid, pid + "h", "HP-12 Rotor pinion 10T 10DP steel", C_ST, "Steel");
            }
            motor(context, id, "rmot", "HP-13 Rotor motor", P(px, py, 1.85), vector(0, 0, 1), true);
            partTube(context, id, "rmnt", "HP-14 Rotor motor clamp collar 1/4 6061 (to bellypan standoffs)", P(px, py, 3.0), P(px, py, 3.25), 1.6, 1.2, C_AL, "Aluminum 6061");
        }
        // exit lift blade: fixed UHMW blade on the pocket circle (r 6.0) ahead of the rear pocket.
        // The rotor turns CCW, so each FUEL rides up the blade (135 -> 180 deg) into the uptake nip.
        {
            const pid = id + "blade";
            const c = new box(0);
            const R0 = 5.81;
            const R1 = 6.19;
            for (var i = 0; i < 11; i += 1)
            {
                const a0 = (135 + 4.5 * i) * degree;
                const a1 = (135 + 4.5 * (i + 1)) * degree;
                const zt = i < 10 ? ROTZ + 0.65 + (7.6 - ROTZ - 0.65) * (i + 1) / 10 : 7.6;
                sPrism(context, pid, c, "XY", [[ROTX + R0 * cos(a0), R0 * sin(a0)], [ROTX + R1 * cos(a0), R1 * sin(a0)],
                            [ROTX + R1 * cos(a1), R1 * sin(a1)], [ROTX + R0 * cos(a1), R0 * sin(a1)]], ROTZ + 0.35, zt);
            }
            done(context, pid, pid + "h", "HP-15 Exit lift blade 3/8 UHMW (arc, rides in the fin slots)", C_WHT, "HDPE");
        }
        // uptake: two vertical belt runs at X = TURX -/+ UPB (FUEL enters the nip sideways off the
        // blade), 4 pulleys on Y-axis shafts carried by 4 struts hung from the deck
        for (var xs in [1, -1])
        {
            const f = xs > 0 ? "F" : "B";
            for (var zz in [UPZ0, UPZ1])
            {
                partCyl(context, id, "pul" ~ f ~ (zz > 10 ? "T" : "B"), "HP-18 Uptake pulley 24T 5mm HTD " ~ f,
                    P(TURX + xs * UPP, -0.3, zz), P(TURX + xs * UPP, 0.3, zz), 0.72, C_AL, "Aluminum 6061");
                if (!(xs > 0 && zz > 10))
                    partCyl(context, id, "ush" ~ f ~ (zz > 10 ? "T" : "B"), "HP-21 Uptake shaft 3/8 hex " ~ f,
                        P(TURX + xs * UPP, -0.9, zz), P(TURX + xs * UPP, 0.9, zz), 0.19, C_ST, "Steel");
            }
            partBox(context, id, "belt" ~ f, "HP-19 Uptake belt 15 mm HTD (rubber-backed) " ~ f,
                P(TURX + xs * UPB, -0.3, UPZ0), P(TURX + xs * (UPB + 0.15), 0.3, UPZ1), C_BLK, "Rubber");
            partBox(context, id, "beltR" ~ f, "HP-20 Uptake belt return " ~ f,
                P(TURX + xs * (UPP + 0.72), -0.3, UPZ0), P(TURX + xs * (UPP + 0.87), 0.3, UPZ1), C_BLK, "Rubber");
            for (var sy in [1, -1])
                partBox(context, id, "strut" ~ f ~ (sy > 0 ? "L" : "R"), "HP-16 Uptake strut 1/4 6061 " ~ f ~ (sy > 0 ? "L" : "R"),
                    P(TURX + xs * 3.2, sy * 0.5, 7.6), P(TURX + xs * 3.9, sy * 0.75, DECKZ + 0.125), C_AL, "Aluminum 6061");
        }
        partCyl(context, id, "ushD", "HP-21 Uptake drive shaft 3/8 hex (front top pulley to motor)", P(TURX + UPP, -0.9, UPZ1), P(TURX + UPP, 9.55, UPZ1), 0.19, C_ST, "Steel");
        motor(context, id, "umot", "HP-22 Uptake motor", P(TURX + UPP, 12.7, UPZ1), vector(0, -1, 0), true);
        partBox(context, id, "ucan", "HP-23 CANrange (uptake ball detect)", P(TURX - 4.0, 0.8, 9.8), P(TURX - 3.3, 1.5, 10.6), C_BLK, "Electronics");
        partBox(context, id, "fcan", "HP-24 CANrange (hopper fill level)", P(3.6, -0.7, DECKZ - 0.6), P(4.4, 0.7, DECKZ), C_BLK, "Electronics");
        // turret fence: 1/16 polycarb wall between hopper and turret sweep. Flat at X = 4.4
        // out to the side walls, bulging forward as an R 9.55 arc around the turret axis
        // (rotating group swing radius 9.36) so FUEL cannot reach the turret.
        {
            const pid = id + "fence";
            const hid = id + "fenceh";
            const c = new box(0);
            const R = FENR;
            const W = FY - 0.0625;
            const th = acos((4.4 - TURX) / R);
            var pts = [[4.4, W], [4.4, R * sin(th)]];
            for (var i = 1; i < 16; i += 1)
            {
                const a = th - 2 * th * i / 16;
                pts = append(pts, [TURX + R * cos(a), R * sin(a)]);
            }
            pts = append(pts, [4.4, -R * sin(th)]);
            pts = append(pts, [4.4, -W]);
            pts = append(pts, [4.4625, -W]);
            pts = append(pts, [4.4625, -(R + 0.0625) * sin(th)]);
            for (var i = 15; i > 0; i -= 1)
            {
                const a = th - 2 * th * i / 16;
                pts = append(pts, [TURX + (R + 0.0625) * cos(a), (R + 0.0625) * sin(a)]);
            }
            pts = append(pts, [4.4625, (R + 0.0625) * sin(th)]);
            pts = append(pts, [4.4625, W]);
            sPrism(context, pid, c, "XY", pts, DECKZ + 0.25, 21.0);
            done(context, pid, hid, "HP-27 Turret fence 1/16 polycarb (arc R 9.55 about turret axis)", C_PC, "Polycarbonate");
        }
    });

/* =====================================================================
 * 04 INTAKE: pivoting over-bumper intake. The arm pivots at O (X 12.75,
 * Z 10.21) just inside the frame front. Deployed, a 37 deg belted ramp runs
 * from a 1.5 in front roller (8.6 in beyond the frame) up over the bumper to
 * the pivot; FUEL is lifted over the front roller onto six 15 mm grip belts
 * and held down by two 1 in idler rollers. Folded 127 deg it stands vertically
 * inside the frame front and closes the hopper. Roller X44 rides on the arm
 * above the FUEL path (service loop at the pivot); deploy X44 is frame mounted
 * (25:1 MAXPlanetary + 2:1 belt) driving a pulley keyed to the arm.
 * ===================================================================== */
annotation { "Feature Type Name" : "APEX 04 Intake" }
export const apexIntake = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Deployed" }
        definition.deployed is boolean;
    }
    {
        const aid = id + "arm";      // everything that pivots with the arm
        // arm side plates, 1/4 7075, pocketed
        for (var sy in [1, -1])
        {
            const k = sy > 0 ? "L" : "R";
            const pid = aid + ("pl" ~ k);
            const hid = aid + ("plh" ~ k);
            const c = new box(0);
            var pts = [];
            for (var q in [[-0.7, -0.3], [9.6, -0.3], [10.0, -0.75], [11.5, -0.75], [11.5, 1.2], [9.6, 8.1], [7.6, 8.1], [1.0, 7.2], [-0.7, 2.0]])
                pts = append(pts, aP(q[0], q[1]));
            sPrism(context, pid, c, "XZ", pts, sy > 0 ? IYP0 : -IYP1, sy > 0 ? IYP1 : -IYP0);
            for (var s0 in [2.9, 5.6])
            {
                var hp = [];
                for (var q in [[s0, 0.8], [s0 + 2.1, 0.8], [s0 + 2.1, 5.2], [s0, 5.2]])
                    hp = append(hp, aP(q[0], q[1]));
                sPrism(context, hid, c, "XZ", hp, -14, 14);
            }
            done(context, pid, hid, "IN-0" ~ (sy > 0 ? 1 : 2) ~ " Intake arm plate " ~ k ~ " 1/4 7075 (pocketed)", C_AL, "Aluminum 7075");
        }
        partCyl(context, aid, "piv", "IN-03 Arm pivot shaft 1/2 hex 7075", vXZ(aP(0, 0), -IYP1), vXZ(aP(0, 0), IYP1), 0.25, C_AL, "Aluminum 7075");
        partTube(context, aid, "rF", "IN-04 Front roller 1.5 in polycarb + silicone sleeve", vXZ(aP(IL, 0), -IYW), vXZ(aP(IL, 0), IYW), 0.75, 0.6, C_SIL, "Polycarbonate");
        {
            const pid = aid + "bed";
            const c = new box(0);
            var pts = [];
            for (var q in [[0.95, -0.275], [9.9, -0.275], [9.9, -0.15], [0.95, -0.15]])
                pts = append(pts, aP(q[0], q[1]));
            sPrism(context, pid, c, "XZ", pts, -IYW, IYW);
            done(context, pid, pid + "h", "IN-05 Ramp bed 1/8 polycarb (PTFE tape)", C_PC, "Polycarbonate");
        }
        var bi = 0;
        for (var yc in [-9.6, -5.8, -1.9, 1.9, 5.8, 9.6])
        {
            bi += 1;
            const pid = aid + ("belt" ~ bi);
            const c = new box(0);
            var pts = [];
            for (var q in [[0.95, -0.15], [9.9, -0.15], [9.9, 0.0], [0.95, 0.0]])
                pts = append(pts, aP(q[0], q[1]));
            sPrism(context, pid, c, "XZ", pts, yc - 0.3, yc + 0.3);
            done(context, pid, pid + "h", "IN-06 Ramp grip belt 15 mm #" ~ bi, C_BLK, "Rubber");
        }
        partTube(context, aid, "rT", "IN-07 Hold-down roller 1 in (over the bumper, idler)", vXZ(aP(1.7, 6.01), -IYW), vXZ(aP(1.7, 6.01), IYW), 0.5, 0.4, C_SIL, "Polycarbonate");
        for (var sy in [1, -1])
            partTube(context, aid, "rT2" ~ (sy > 0 ? "L" : "R"), "IN-08 Hold-down roller 1 in " ~ (sy > 0 ? "L" : "R") ~ " (split, idler)",
                vXZ(aP(7.5, 6.01), sy > 0 ? 6.2 : -IYW), vXZ(aP(7.5, 6.01), sy > 0 ? IYW : -6.2), 0.5, 0.4, C_SIL, "Polycarbonate");
        partCyl(context, aid, "tie", "IN-09 Arm tie tube 1/2 in 7075", vXZ(aP(2.5, 6.5), -IYP0), vXZ(aP(2.5, 6.5), IYP0), 0.25, C_AL, "Aluminum 7075");
        partCyl(context, aid, "idl", "IN-10 Belt return idler 0.6 in", vXZ(aP(0.6, -0.3), -IYW), vXZ(aP(0.6, -0.3), IYW), 0.3, C_AL, "Aluminum 6061");
        // roller drive: X44 on the arm above the FUEL path, belt down to the front roller (left side)
        motor(context, aid, "mRol", "IN-14 Intake roller motor (arm-mounted, 1.87:1 belt)", vXZ(aP(8.8, 6.9), 9.4), vector(0, 1, 0), false);
        partCyl(context, aid, "pulF", "IN-15 Front roller pulley 28T", vXZ(aP(IL, 0), 11.55), vXZ(aP(IL, 0), 11.9), 0.5, C_AL, "Aluminum 6061");
        {
            const pid = aid + "abelt";
            const c = new box(0);
            var pts = [];
            for (var q in [[9.185, 7.008], [11.115, 0.108], [10.345, -0.108], [8.415, 6.792]])
                pts = append(pts, aP(q[0], q[1]));
            sPrism(context, pid, c, "XZ", pts, 11.6, 11.9);
            done(context, pid, pid + "h", "IN-11 Roller belt 9 mm HTD (arm motor -> front roller)", C_BLK, "Rubber");
        }
        partCyl(context, aid, "pulL", "IN-12 Deploy pulley 24T (keyed to the arm)", vXZ(aP(0, 0), 12.25), vXZ(aP(0, 0), 12.55), 0.75, C_AL, "Aluminum 6061");
        if (!definition.deployed)
            rotY(context, id + "fold", qCreatedBy(aid, EntityType.BODY), IOX, IOZ, -127 * degree);

        // frame-mounted drive (does not move with the arm)
        motor(context, id, "mDep", "IN-13 Intake deploy motor (25:1 MAXPlanetary)", P(11.2, 9.95, 10.3), vector(0, 1, 0), false);
        {
            const pid = id + "fbL";
            const c = new box(0);
            sPrism(context, pid, c, "XZ", [[11.2, 10.68], [IOX, IOZ + 0.75], [IOX + 0.75, IOZ], [IOX, IOZ - 0.75], [11.2, 9.92], [10.82, 10.3]], 12.25, 12.55);
            done(context, pid, pid + "h", "IN-16 Deploy belt 9 mm HTD (12T -> 24T)", C_BLK, "Rubber");
        }
    });

/* =====================================================================
 * 05 TURRET SHOOTER: 9 in ID x-contact bearing, 120T 10DP ring, X44 72:1,
 * single-stream hooded flywheel (3x 4 in wheels), 2x X60 1:1, steel inertia
 * disc 2.2:1, static PTFE hood on X44 100:1 (launch 50-80 deg), LL4.
 * Local frame: ball enters at the turret axis, launches toward +x_local.
 * ===================================================================== */
annotation { "Feature Type Name" : "APEX 05 Turret Shooter" }
export const apexTurret = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Turret angle (0 = shooting over the rear)" }
        isAngle(definition.turret, ANGLE_360_ZERO_DEFAULT_BOUNDS);
        annotation { "Name" : "Launch angle (45-80 deg)" }
        isAngle(definition.launch, { (degree) : [45, 65, 80] } as AngleBoundSpec);
    }
    {
        // fixed parts: bearing outer race + clock-spring cable wrap + yaw motor
        partTube(context, id, "brgO", "TS-01 Bearing 9 x 10.25 in thin-section, outer race", P(TURX, 0, DECKZ + 0.25), P(TURX, 0, DECKZ + 0.75), 5.125, 4.85, C_ST, "Steel");
        partTube(context, id, "wrapC", "TS-02 Clock-spring cable wrap housing (+/-200 deg)", P(TURX, 0, DECKZ + 0.8), P(TURX, 0, DECKZ + 1.2), 8.6, 6.4, C_PCDK, "Nylon");
        const ya = 150 * degree;
        const ymx = TURX + 6.6 * cos(ya);
        const ymy = 6.6 * sin(ya);
        motor(context, id, "yaw", "TS-03 Turret yaw motor (6:1 MAXPlanetary)", P(ymx, ymy, DECKZ - 0.4), vector(0, 0, -1), false);
        {
            const pid = id + "ypin";
            const c = new box(0);
            sGear(context, pid, c, ymx, ymy, 10, 10, DECKZ + 0.25, DECKZ + 0.75);
            done(context, pid, pid + "h", "TS-04 Yaw pinion 10T 10DP", C_ST, "Steel");
        }
        partBox(context, id, "tcc", "TS-05 Turret CANcoder (17:20 vernier gear)", P(TURX + 5.3, -0.75, DECKZ - 0.7), P(TURX + 6.5, 0.75, DECKZ), C_BLK, "Electronics");

        // rotating parts are built in a frame where +X is the launch direction at turret = 0
        // (turret 0 = shooting over the rear: local +x maps to robot -X)
        const rid = id + "rot";
        const ox = TURX;
        // bearing inner race + 120T ring gear + rotating plate
        partTube(context, rid, "brgI", "TS-06 Bearing inner race", P(ox, 0, DECKZ + 0.75), P(ox, 0, DECKZ + 1.0), 4.85, 4.5, C_ST, "Steel");
        {
            const pid = rid + "ring";
            const hid = rid + "ringh";
            const c = new box(0);
            sGear(context, pid, c, ox, 0, 120, 10, DECKZ + 0.25, DECKZ + 0.5);
            hCyl(context, hid, c, P(ox, 0, DECKZ), P(ox, 0, DECKZ + 1), 5.25);
            done(context, pid, hid, "TS-07 Turret ring gear 120T 10DP 1/4 7075", C_AL, "Aluminum 7075");
        }
        {
            const pid = rid + "tplate";
            const hid = rid + "tplateh";
            const c = new box(0);
            sCyl(context, pid, c, P(ox, 0, DECKZ + 1.0), P(ox, 0, DECKZ + 1.25), 5.6);
            hCyl(context, hid, c, P(ox, 0, DECKZ), P(ox, 0, DECKZ + 2), 4.3);
            done(context, pid, hid, "TS-08 Turret plate 1/4 6061", C_AL, "Aluminum 6061");
        }
        // shooter side plates (local: x from -3.6 to 8.9 along launch dir, y +/-3.45)
        const zb = DECKZ + 1.25;
        for (var sy in [1, -1])
        {
            const k = sy > 0 ? "L" : "R";
            const pid = rid + ("sp" ~ k);
            const hid = rid + ("sph" ~ k);
            const c = new box(0);
            // polygon in XZ (local x = -robot X direction handled by rotation later)
            sPrism(context, pid, c, "XZ", [[ox - 3.6, zb], [ox + 8.6, zb], [ox + 8.6, 18.0], [ox + 7.8, 21.6], [ox + 5.0, 21.6], [ox - 1.0, 19.6], [ox - 3.6, 17.2]],
                sy > 0 ? 3.45 : -3.70, sy > 0 ? 3.70 : -3.45);
            hCyl(context, hid, c, P(ox + FLYX, -5, FLYZ), P(ox + FLYX, 5, FLYZ), 0.55);
            hCyl(context, hid, c, P(ox + 6.8, -5, 19.4), P(ox + 6.8, 5, 19.4), 0.30);
            hCyl(context, hid, c, P(ox + 7.6, -5, 15.05), P(ox + 7.6, 5, 15.05), 0.5);
            hBox(context, hid, c, P(ox + 0.3, -5, zb + 0.7), P(ox + 2.3, 5, 17.0));
            hBox(context, hid, c, P(ox + 2.9, -5, 17.4), P(ox + 6.0, 5, 20.3));
            hBox(context, hid, c, P(ox - 3.0, -5, zb + 0.6), P(ox - 0.4, 5, 16.4));
            done(context, pid, hid, "TS-1" ~ (sy > 0 ? 0 : 1) ~ " Shooter side plate " ~ k ~ " 1/4 6061", C_AL, "Aluminum 6061");
        }
        // flywheel: 3x 4 in 60A wheels on 1/2 hex shaft
        {
            const pid = rid + "fly";
            const c = new box(0);
            for (var yy in [-2.25, 0.0, 2.25])
                sCyl(context, pid, c, P(ox + FLYX, yy - 0.9, FLYZ), P(ox + FLYX, yy + 0.9, FLYZ), 2.0);
            done(context, pid, pid + "h", "TS-12 Flywheel: 3x 4 in 60A urethane wheels", C_ORG, "Rubber");
        }
        partCyl(context, rid, "flysh", "TS-13 Flywheel shaft 1/2 hex", P(ox + FLYX, -4.3, FLYZ), P(ox + FLYX, 4.3, FLYZ), 0.25, C_ST, "Steel");
        partCyl(context, rid, "mass", "TS-14 Inertia disc 4.5 in x 0.33 in 1018 steel, 1.5 lb (2.2:1 overdrive)", P(ox + 6.8, -0.165, 19.4), P(ox + 6.8, 0.165, 19.4), 2.25, C_ST, "Steel");
        partCyl(context, rid, "masssh", "TS-15 Inertia jackshaft 3/8 hex", P(ox + 6.8, -4.2, 19.4), P(ox + 6.8, 4.2, 19.4), 0.19, C_ST, "Steel");
        motor(context, rid, "fm1", "TS-16 Flywheel motor L", P(ox + 7.6, 0.45, 15.05), vector(0, 1, 0), true);
        motor(context, rid, "fm2", "TS-17 Flywheel motor R", P(ox + 7.6, -0.45, 15.05), vector(0, -1, 0), true);
        for (var sy in [1, -1])
        {
            const k = sy > 0 ? "L" : "R";
            partBox(context, rid, "blt" ~ k, "TS-18 Flywheel belt 9 mm HTD " ~ k, P(ox + 4.2, sy * 3.85, 15.0), P(ox + 7.5, sy * 4.15, 16.0), C_BLK, "Rubber");
            partBox(context, rid, "blm" ~ k, "TS-19 Inertia belt 9 mm HTD " ~ k, P(ox + 4.3, sy * 3.85, 16.1), P(ox + 6.9, sy * 4.15, 19.5), C_BLK, "Rubber");
        }
        // hood: static PTFE-lined arc concentric with the flywheel, pivoting about it.
        // release angle phi = 180 - (90 - launch)... ball path radius 4.35, hood inner radius 7.3
        {
            const pid = rid + "hood";
            const c = new box(0);
            const rel = 90 * degree + definition.launch;   // polar angle of the release point
            var pts = [];
            const a0 = rel;
            const a1 = rel + 31 * degree;
            for (var i = 0; i <= 12; i += 1)
            {
                const a = a0 + (a1 - a0) * i / 12;
                pts = append(pts, [ox + FLYX + 7.30 * cos(a), FLYZ + 7.30 * sin(a)]);
            }
            for (var i = 12; i >= 0; i -= 1)
            {
                const a = a0 + (a1 - a0) * i / 12;
                pts = append(pts, [ox + FLYX + 7.55 * cos(a), FLYZ + 7.55 * sin(a)]);
            }
            sPrism(context, pid, c, "XZ", pts, -3.3, 3.3);
            done(context, pid, pid + "h", "TS-20 Hood 1/4 polycarb + PTFE liner (pivots on flywheel axis)", C_PC, "Polycarbonate");
        }
        {
            // hood sector (left side, outside the plate): toothed annulus r 4.7 - 5.45 about the
            // flywheel axis spanning rel + 5 .. rel + 50 deg, with a spoke out to the hood. The X44
            // pinion sits at 180 deg (pitch-line contact r 5.7), engaged for every launch angle 45-80.
            const pid = rid + "hsec";
            const c = new box(0);
            const rel = 90 * degree + definition.launch;
            var pts = [];
            for (var i = 0; i <= 12; i += 1)
            {
                const a = rel + 5 * degree + 45 * degree * i / 12;
                pts = append(pts, [ox + FLYX + 5.45 * cos(a), FLYZ + 5.45 * sin(a)]);
            }
            for (var i = 12; i >= 0; i -= 1)
            {
                const a = rel + 5 * degree + 45 * degree * i / 12;
                pts = append(pts, [ox + FLYX + 4.7 * cos(a), FLYZ + 4.7 * sin(a)]);
            }
            sPrism(context, pid, c, "XZ", pts, 3.75, 4.0);
            var sp = [];
            for (var q in [[5.3, 3.5], [7.4, 3.5], [7.4, 7.5], [5.3, 7.5]])
            {
                const a = rel + q[1] * degree;
                sp = append(sp, [ox + FLYX + q[0] * cos(a), FLYZ + q[0] * sin(a)]);
            }
            sPrism(context, pid, c, "XZ", sp, 3.75, 4.0);
            done(context, pid, pid + "h", "TS-21 Hood sector 1/4 7075 (10DP teeth, 45 deg)", C_AL, "Aluminum 7075");
        }
        motor(context, rid, "hm", "TS-22 Hood motor (100:1, homes on the 80 deg hard stop)", P(ox + FLYX - 5.7, 6.45, FLYZ), vector(0, -1, 0), false);
        partBox(context, rid, "ll", "TS-23 Limelight 4 (turret, faces launch direction, hub tags)", P(ox + 3.0, 4.2, 17.5), P(ox + 5.2, 5.3, 20.9), C_BLK, "Electronics");
        // rotate the whole rotating group: turret 0 -> launch toward robot rear (-X)
        rotZ(context, id + "spin", qCreatedBy(rid, EntityType.BODY), TURX, 0, 180 * degree + definition.turret);
    });

/* =====================================================================
 * 06 ELECTRICAL: PDH, roboRIO 2, VH-109, CANivore, switch, MPM, Pigeon 2,
 * 120 A main breaker, SB50, battery, RSL, 3 chassis Limelight 4s.
 * ===================================================================== */
annotation { "Feature Type Name" : "APEX 06 Electrical" }
export const apexElectrical = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        const z = RZ0;
        partBox(context, id, "pdh", "EL-01 REV Power Distribution Hub", P(-12.3, -3.7, z), P(-7.9, 3.7, z + 1.85), C_BLK, "Electronics");
        partBox(context, id, "tray", "EL-02 Battery tray 1/8 6061 (slides out rear)", P(-FX + 1.0, -8.0, RZ1), P(-6.8, -0.4, RZ1 + 0.125), C_AL, "Aluminum 6061");
        partBox(context, id, "batt", "EL-03 Battery 12 V 18 Ah SLA (on edge)", P(-12.4, -7.6, RZ1 + 0.125), P(-9.4, -0.5, RZ1 + 6.725), C_BLK, "Battery");
        partBox(context, id, "strap", "EL-04 Battery strap + latch (over the top)", P(-12.6, -4.6, RZ1 + 6.725), P(-9.2, -3.4, RZ1 + 6.975), C_GRY, "Nylon");
        partBox(context, id, "sb50", "EL-05 Anderson SB50 (red)", P(-11.8, -2.6, RZ1 + 6.8), P(-10.0, -1.4, RZ1 + 7.7), C_RED, "Nylon");
        partBox(context, id, "brk", "EL-06 120 A main breaker (rear access)", P(-12.4, 0.5, RZ1 + 0.2), P(-10.3, 3.4, RZ1 + 2.3), C_BLK, "Electronics");
        partBox(context, id, "radio", "EL-07 Vivid-Hosting VH-109 radio (LEDs visible rear)", P(-12.6, 4.5, 5.5), P(-11.3, 9.5, 10.5), C_WHT, "Electronics");
        partBox(context, id, "rio", "EL-08 NI roboRIO 2.0", P(6.0, -2.7, z), P(11.7, 2.7, z + 1.4), C_GRY, "Electronics");
        partBox(context, id, "cniv", "EL-09 CTRE CANivore (CAN FD bus 2)", P(0.5, 4.5, z), P(3.25, 6.1, z + 0.75), C_BLK, "Electronics");
        partBox(context, id, "sw", "EL-10 5-port Ethernet switch (12 V)", P(0.5, -6.5, z), P(4.0, -4.0, z + 0.9), C_BLK, "Electronics");
        partBox(context, id, "mpm", "EL-11 REV Mini Power Module (camera/switch power)", P(-4.5, 4.5, z), P(-1.5, 6.4, z + 0.9), C_BLK, "Electronics");
        partBox(context, id, "pig", "EL-12 CTRE Pigeon 2.0 IMU", P(-2.5, -1.0, z), P(-0.6, 1.0, z + 0.7), C_BLK, "Electronics");
        partCyl(context, id, "rsl", "EL-13 Robot Signal Light (am-3583)", P(-12.5, 12.4, 21.0 - 2.4), P(-12.5, 12.4, 21.0 - 0.3), 1.0, C_ORG, "Electronics");
        // chassis Limelight 4s
        partBox(context, id, "llL", "EL-14 Limelight 4 left (AprilTags)", P(-2.0, FY - 1.4, 17.8), P(1.4, FY - 0.15, 20.0), C_BLK, "Electronics");
        partBox(context, id, "llR", "EL-15 Limelight 4 right (AprilTags)", P(-2.0, -FY + 0.15, 17.8), P(1.4, -FY + 1.4, 20.0), C_BLK, "Electronics");
        partBox(context, id, "llF", "EL-16 Limelight 4 + Hailo-8 front (FUEL detection)", P(6.15, -1.7, 19.3), P(7.25, 1.7, 20.7), C_BLK, "Electronics");
        partBox(context, id, "rioM", "EL-17 Electronics standoffs / brainpan rails", P(-0.5, -7.0, z), P(0.0, 7.0, z + 0.3), C_AL, "Aluminum 6061");
    });

/* =====================================================================
 * 07 WIRING: colour-coded runs. Red/black = power (6/12/14/18 AWG),
 * yellow = CAN bus 1 (roboRIO), orange = CAN FD bus 2 (CANivore),
 * blue = Ethernet, white = USB. Waypoints follow frame rails.
 * ===================================================================== */
annotation { "Feature Type Name" : "APEX 07 Wiring" }
export const apexWiring = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        const r6 = 0.17;
        const r12 = 0.095;
        const r14 = 0.08;
        const r18 = 0.06;
        const rcan = 0.07;
        const reth = 0.11;
        // main power: battery -> SB50 -> breaker -> PDH
        wire(context, id, "w01", "WR-01 Battery + to SB50 (6 AWG red)", [[-10.6, -2.2, 10.35], [-10.6, -2.2, 11.0], [-10.9, -2.2, 11.2]], r6, C_RED);
        wire(context, id, "w02", "WR-02 Battery - to SB50 (6 AWG black)", [[-10.6, -5.9, 10.35], [-10.6, -5.9, 11.0], [-10.9, -2.0, 11.2]], r6, C_BLK);
        wire(context, id, "w03", "WR-03 SB50 + to 120 A breaker (6 AWG red)", [[-11.0, -2.2, 11.2], [-12.7, -1.0, 10.6], [-12.7, 1.0, 6.4], [-11.4, 1.2, 5.8]], r6, C_RED);
        wire(context, id, "w04", "WR-04 Breaker to PDH + input (6 AWG red)", [[-11.4, 2.8, 5.8], [-12.7, 2.9, 5.0], [-12.7, 2.6, 3.0], [-12.2, 2.4, 2.4]], r6, C_RED);
        wire(context, id, "w05", "WR-05 SB50 - to PDH - input (6 AWG black)", [[-11.2, -2.0, 11.2], [-12.8, -0.4, 10.4], [-12.8, -0.4, 3.6], [-12.8, -1.9, 3.0], [-12.2, -2.2, 2.4]], r6, C_BLK);
        // PDH -> swerve modules (12 AWG drive pairs, 14 AWG steer pairs), along the side rails
        var k = 10;
        for (var sx in [1, -1])
            for (var sy in [1, -1])
            {
                const mx = sx * MODX;
                const my = sy * MODY;
                const tg = (sx > 0 ? "F" : "B") ~ (sy > 0 ? "L" : "R");
                const yrun = sy * 12.2;
                for (var pol in [0, 1])
                {
                    k += 1;
                    const off = pol == 0 ? 0.2 : -0.2;
                    const col = pol == 0 ? C_RED : C_BLK;
                    wire(context, id, "w" ~ k, "WR-" ~ k ~ " PDH -> drive " ~ tg ~ (pol == 0 ? " + (12 AWG red)" : " - (12 AWG black)"),
                        [[-8.0, sy * 3.0 + off, 2.6], [-6.5, yrun + off, 2.6], [mx - sx * 3.0, yrun + off, 3.9], [mx - sx * 1.0, my - sy * 1.5 + off, 5.0], [mx, my - sy * 1.2, 5.6]], r12, col);
                }
                k += 1;
                wire(context, id, "w" ~ k, "WR-" ~ k ~ " PDH -> steer " ~ tg ~ " (14 AWG pair)", [[-8.0, sy * 3.4, 2.4], [-6.0, yrun - sy * 0.5, 2.4], [mx - sx * 3.6, yrun - sy * 0.5, 3.9], [mx - sx * 2.55, my - sy * 0.9, 5.4]], r14, C_RED);
            }
        // PDH -> rotor, uptake, intake (via carrier), turret wrap bundle
        wire(context, id, "w30", "WR-30 PDH -> rotor X60 (12 AWG pair)", [[-7.9, -3.0, 2.6], [0.0, -7.6, 2.6], [8.0, -7.8, 2.6], [8.75, -6.4, 2.4]], r12, C_RED);
        wire(context, id, "w31", "WR-31 PDH -> uptake X60 (12 AWG pair)", [[-7.9, 2.5, 2.8], [-6.5, 13.4, 4.0], [-1.0, 13.4, 11.0], [-0.03, 13.1, 11.5]], r12, C_RED);
        wire(context, id, "w32", "WR-32 PDH -> intake deploy X44 (14 AWG pair)", [[-7.9, 3.0, 2.6], [-6.0, 13.4, 3.9], [9.0, 13.4, 8.6], [10.6, 9.4, 9.6], [11.2, 9.5, 10.3]], r14, C_RED);
        wire(context, id, "w35", "WR-35 PDH -> intake roller X44 (14 AWG pair, service loop at the pivot)", [[-7.9, 3.4, 2.6], [-6.0, 13.3, 4.2], [9.0, 13.3, 8.9], [12.2, 12.4, 10.6]], r14, C_RED);
        wire(context, id, "w33", "WR-33 PDH -> turret bundle (2x 12 AWG fly, 14 AWG hood, CAN FD, Eth, LL power) to clock-spring", [[-8.0, 0.0, 3.4], [-8.6, 0.0, 6.0], [-8.6, 4.0, 11.6], [-6.0, 7.4, DECKZ + 1.0]], 0.24, C_BLK);
        wire(context, id, "w34", "WR-34 PDH -> turret yaw X44 (14 AWG pair)", [[-8.2, 1.5, 3.4], [-9.2, 3.0, 6.0], [TURX + 6.6 * cos(150 * degree), 6.6 * sin(150 * degree), DECKZ - 2.6]], r14, C_RED);
        // control power
        wire(context, id, "w40", "WR-40 PDH ch20 (10 A) -> roboRIO 2 (18 AWG pair)", [[-7.9, 1.0, 2.0], [-4.0, 2.0, 1.7], [5.0, 2.0, 1.7], [6.0, 1.5, 1.9]], r18, C_RED);
        wire(context, id, "w41", "WR-41 PDH ch21 (10 A) -> VH-109 12 V input (18 AWG pair)", [[-9.5, 3.7, 2.8], [-11.0, 4.4, 4.0], [-11.9, 5.0, 5.6]], r18, C_RED);
        wire(context, id, "w42", "WR-42 PDH ch22 -> MPM (18 AWG pair)", [[-7.9, 2.2, 2.0], [-6.0, 4.0, 1.8], [-4.5, 5.4, 1.9]], r18, C_RED);
        wire(context, id, "w43", "WR-43 MPM -> Limelight left (18 AWG pair)", [[-3.0, 6.4, 2.0], [-2.0, 12.4, 3.9], [-1.0, 12.4, 17.6], [-0.3, 13.0, 18.0]], r18, C_RED);
        wire(context, id, "w44", "WR-44 MPM -> Limelight right (18 AWG pair)", [[-3.0, 4.5, 2.0], [-2.0, -12.4, 3.9], [-1.0, -12.4, 17.6], [-0.3, -13.0, 18.0]], r18, C_RED);
        wire(context, id, "w45", "WR-45 MPM -> Limelight front (18 AWG pair)", [[-1.5, 5.2, 2.0], [3.0, 13.3, 4.0], [4.6, 13.3, 13.5], [4.6, 6.2, 20.6], [5.8, 3.0, 20.6], [6.2, 0.8, 20.3]], r18, C_RED);
        wire(context, id, "w46", "WR-46 MPM -> Ethernet switch (18 AWG pair)", [[-1.5, 4.8, 1.9], [0.0, 0.0, 1.8], [1.0, -4.0, 1.9]], r18, C_RED);
        // CAN bus 1 (roboRIO native): RIO -> PDH -> intake motors -> rotor -> uptake -> CANranges -> terminator
        wire(context, id, "w50", "WR-50 CAN1 roboRIO -> PDH (yellow/green)", [[6.0, -2.0, 1.9], [-4.0, -2.2, 1.7], [-7.9, -2.0, 2.2]], rcan, C_YEL);
        wire(context, id, "w51", "WR-51 CAN1 PDH -> rotor X60 -> intake X44s (deploy; roller via pivot loop) -> uptake X60 -> CANranges", [[-7.9, -3.4, 2.6], [5.0, -7.4, 2.6], [8.7, -6.6, 2.2], [12.3, -6.0, 4.0], [12.3, 9.0, 4.0], [11.2, 9.4, 9.8], [12.2, 12.3, 10.4], [9.0, 13.3, 8.6], [0.5, 13.3, 11.9], [-6.0, 9.0, 12.5], [-7.6, 2.0, 12.5], [-7.15, 1.5, 10.6], [-7.6, 2.0, 12.6], [-4.0, 6.0, 12.65], [3.9, 0.8, 12.6]], rcan, C_YEL);
        // CAN FD bus 2 (CANivore): modules + Pigeon + turret group
        wire(context, id, "w52", "WR-52 USB roboRIO -> CANivore (white)", [[6.0, 2.2, 2.2], [4.0, 4.0, 2.2], [3.25, 5.3, 1.9]], 0.09, C_WHT);
        wire(context, id, "w53", "WR-53 CAN FD loop: CANivore -> FL -> BL -> BR -> FR -> Pigeon -> turret wrap", [[0.5, 5.3, 1.9], [6.0, 11.9, 3.0], [MODX, MODY - 1.6, 4.0], [0.0, 12.0, 3.0], [-MODX, MODY - 1.6, 4.0], [-9.0, 0.0, 3.6], [-MODX, -MODY + 1.6, 4.0], [0.0, -12.0, 3.0], [MODX, -MODY + 1.6, 4.0], [5.0, -9.0, 2.2], [-1.0, -1.0, 2.1], [-8.6, 0.3, 6.5], [-6.2, 7.0, DECKZ + 1.0]], rcan, C_ORG);
        // Ethernet
        wire(context, id, "w60", "WR-60 Ethernet VH-109 RIO port -> roboRIO", [[-11.3, 6.0, 7.0], [-8.0, 6.5, 3.0], [5.0, 3.5, 2.0], [6.0, 2.4, 2.2]], reth, C_ETH);
        wire(context, id, "w61", "WR-61 Ethernet VH-109 AUX -> switch", [[-11.3, 8.0, 7.0], [-6.0, 9.5, 3.5], [0.0, -2.0, 2.2], [0.5, -5.2, 2.0]], reth, C_ETH);
        wire(context, id, "w62", "WR-62 Ethernet switch -> Limelight left", [[2.0, -4.0, 2.2], [2.0, 12.6, 4.0], [1.0, 12.6, 18.4]], reth, C_ETH);
        wire(context, id, "w63", "WR-63 Ethernet switch -> Limelight right", [[2.5, -6.5, 2.2], [2.5, -12.6, 4.0], [1.0, -12.6, 18.4]], reth, C_ETH);
        wire(context, id, "w64", "WR-64 Ethernet switch -> Limelight front", [[4.0, -5.0, 2.2], [3.5, -13.3, 4.0], [4.6, -13.3, 13.5], [4.6, -6.2, 20.4], [5.8, -3.0, 20.4], [6.2, -0.8, 20.0]], reth, C_ETH);
        wire(context, id, "w65", "WR-65 Ethernet switch -> turret Limelight (via clock-spring)", [[3.0, -4.0, 2.4], [-5.0, -3.0, 3.8], [-8.4, 0.6, 6.0], [-6.4, 7.2, DECKZ + 1.0]], reth, C_ETH);
        // RSL
        wire(context, id, "w70", "WR-70 roboRIO RSL output -> RSL (18 AWG)", [[11.0, 2.0, 2.0], [11.0, 12.3, 3.9], [-12.0, 12.3, 4.0], [-12.5, 12.4, 18.6]], r18, C_ORG);
    });

/* =====================================================================
 * CLEARANCE CHECK: builds every subsystem at the given mechanism state,
 * then reports (FeatureScript notices) every interference between parts of
 * different subsystems, the overall envelope, and R104/R105 numbers.
 * Wiring is left out (cables terminate inside device envelopes by design).
 * ===================================================================== */
function partName(context is Context, q is Query) returns string
{
    try silent
    {
        return getProperty(context, { "entity" : q, "propertyType" : PropertyType.NAME });
    }
    return "?";
}

annotation { "Feature Type Name" : "APEX Clearance check" }
export const apexClearance = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Turret angle" }
        isAngle(definition.turret, ANGLE_360_ZERO_DEFAULT_BOUNDS);
        annotation { "Name" : "Launch angle" }
        isAngle(definition.launch, { (degree) : [45, 65, 80] } as AngleBoundSpec);
        annotation { "Name" : "Intake deployed" }
        definition.deployed is boolean;
        annotation { "Name" : "Rotor angle" }
        isAngle(definition.rotor, ANGLE_360_ZERO_DEFAULT_BOUNDS);
    }
    {
        apexDrivetrain(context, id + "dt", { "steer" : 0 * degree });
        apexBumpers(context, id + "bu", { "red" : false });
        apexHopper(context, id + "hp", { "rotor" : definition.rotor });
        apexIntake(context, id + "in", { "deployed" : definition.deployed });
        apexTurret(context, id + "ts", { "turret" : definition.turret, "launch" : definition.launch });
        apexElectrical(context, id + "el", {});
        const keys = ["dt", "bu", "hp", "in", "ts", "el"];
        var n = 0;
        var rep = [];
        for (var i = 0; i < size(keys); i += 1)
        {
            for (var j = i + 1; j < size(keys); j += 1)
            {
                const hits = evCollision(context, {
                            "tools" : qCreatedBy(id + keys[i], EntityType.BODY),
                            "targets" : qCreatedBy(id + keys[j], EntityType.BODY)
                        });
                for (var h in hits)
                {
                    if (h["type"] == ClashType.INTERFERE || h["type"] == ClashType.TARGET_IN_TOOL || h["type"] == ClashType.TOOL_IN_TARGET)
                    {
                        n += 1;
                        rep = append(rep, "CLASH " ~ partName(context, h["toolBody"]) ~ "  <>  " ~ partName(context, h["targetBody"]));
                        println("CLASH " ~ partName(context, h["toolBody"]) ~ "  <>  " ~ partName(context, h["targetBody"]) ~ "  [" ~ toString(h["type"]) ~ "]");
                    }
                }
            }
        }
        const all = qCreatedBy(id, EntityType.BODY);
        const noBump = qSubtraction(all, qCreatedBy(id + "bu", EntityType.BODY));
        const bAll = evBox3d(context, { "topology" : all, "tight" : true });
        const bIn = evBox3d(context, { "topology" : noBump, "tight" : true });
        rep = append(rep, "CLASHES " ~ n);
        rep = append(rep, "ENV " ~ toString([bAll.minCorner[0] / inch, bAll.maxCorner[0] / inch, bAll.minCorner[1] / inch, bAll.maxCorner[1] / inch, bAll.maxCorner[2] / inch]));
        rep = append(rep, "ROBOT " ~ toString([bIn.minCorner[0] / inch, bIn.maxCorner[0] / inch, bIn.minCorner[1] / inch, bIn.maxCorner[1] / inch, bIn.maxCorner[2] / inch]));
        setVariable(context, "apexReport", rep);
        println("CLASHES " ~ n);
        println("ENVELOPE (in, with bumpers) X " ~ roundToPrecision(bAll.minCorner[0] / inch, 2) ~ " .. " ~ roundToPrecision(bAll.maxCorner[0] / inch, 2) ~
                    "  Y " ~ roundToPrecision(bAll.minCorner[1] / inch, 2) ~ " .. " ~ roundToPrecision(bAll.maxCorner[1] / inch, 2) ~
                    "  Z " ~ roundToPrecision(bAll.minCorner[2] / inch, 2) ~ " .. " ~ roundToPrecision(bAll.maxCorner[2] / inch, 2));
        println("ROBOT (no bumpers) X " ~ roundToPrecision(bIn.minCorner[0] / inch, 2) ~ " .. " ~ roundToPrecision(bIn.maxCorner[0] / inch, 2) ~
                    "  Y " ~ roundToPrecision(bIn.minCorner[1] / inch, 2) ~ " .. " ~ roundToPrecision(bIn.maxCorner[1] / inch, 2) ~
                    "  max height " ~ roundToPrecision(bIn.maxCorner[2] / inch, 2) ~ " (TRENCH 22.25, R107 30.0)");
        println("EXTENSION beyond frame perimeter: front " ~ roundToPrecision(bIn.maxCorner[0] / inch - FX, 2) ~ "  rear " ~ roundToPrecision(-bIn.minCorner[0] / inch - FX, 2) ~
                    "  left " ~ roundToPrecision(bIn.maxCorner[1] / inch - FY, 2) ~ "  right " ~ roundToPrecision(-bIn.minCorner[1] / inch - FY, 2) ~ " (R105: <= 12 in, one side)");
    });
