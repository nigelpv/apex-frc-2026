
// =========================================================================
// 07 ELECTRICAL. Vendor models (MKCad) for every control-system part, placed
// with real mounting hardware:
//  - battery standing in a 1/8 6061 sled on the rear rail, strap retained,
//    removed rearward through the single R401 bumper gap (y -2.95 .. 4.85)
//  - 120 A main breaker in a bent bracket hung in the rear deck pocket,
//    directly above the battery (short 6 AWG run, reset reachable from the
//    top, kept below the turret plate)
//  - two 3/16 polycarb electronics boards hung under the deck on the open
//    sides of the robot (bent 6061 hanger at the back, 1/2 OD post at the
//    front): left = roboRIO 2, network switch, REV MPM; right = REV PDH,
//    CANivore (drivetrain CAN FD bus)
//  - VH-109 radio and network switch on the deck outside the turret's
//    cable-wrap ring, RSL on the deck, Pigeon 2 on the
//    bellypan near the robot centre, 2 rear-corner Limelight 4s on printed
//    15 deg wedges, 2 CANrange ball sensors (uptake + hopper full)
// The turret Limelight 4 is built in the turret feature (it rotates).
// =========================================================================
const TRAYT = 0.1875;               // electronics board thickness
const LL4BACK = [3.29, -0.317, 0.83];   // LL4 model: back-face centre (lens on +Y, mounting face -Y)

// place a vendor model so its local point lp (in) lands on world point wp, local +Y along ny, local +Z along nz
function cotsAt(context is Context, id is Id, build is function, name is string, lb is number, lp is array, ny is Vector, nz is Vector, wp is Vector)
{
    const y = normalize(ny);
    const z = normalize(nz);
    const x = cross(y, z);
    const o = wp - (x * lp[0] + y * lp[1] + z * lp[2]) * inch;
    cots(context, id, build, frameT(o, x, z), name, lb);
    setProperty(context, { "entities" : qCreatedBy(id, EntityType.BODY), "propertyType" : PropertyType.NAME, "value" : name });
}

// holes the electrical layout needs in the turret deck [x, y, d]
function elecDeckHoles() returns array
{
    var h = [];
    for (var b in brkBolts())
        h = append(h, [b[0], b[1], D10]);                                   // breaker bracket
    h = append(h, [RSLX, RSLY, 1.2]);                                       // RSL stem + panel nut
    for (var p in concat(radioMountHoles(), switchMountHoles()))
        h = append(h, [p[0], p[1], D10]);                                   // radio + switch mounts
    h = append(h, [TGROMX, TGROMY, 0.6]);                                   // turret harness grommet
    for (var sy in [-1, 1])
    {
        for (var y in [8.6, 12.1])
            h = append(h, [-12.9, sy * y, D10]);                            // board hangers
        h = append(h, [-1.7, sy * 12.4, D10]);                              // board front posts
        for (var b in ll4FootBolts(sy))
            h = append(h, [b[0], b[1], D10]);                               // rear camera wedges
    }
    return h;
}

// radio (left) and network switch (right rear) sit outside the turret's cable-wrap ring
function radioMountHoles() returns array
{
    return [[-9.15, 9.6], [-9.15, 11.6], [-4.05, 9.6], [-4.05, 11.6]];
}

function switchMountHoles() returns array
{
    return [[-7.025, -12.25], [-7.025, -9.55]];
}

function brkBolts() returns array
{
    return [[-13.2, -1.1], [-13.2, 1.1], [-10.58, -1.1], [-10.58, 1.1]];
}

// bellypan fixings for the electrical layout (battery sled legs)
function elecBellyPts() returns array
{
    return [[-8.9, -2.25], [-8.9, 4.25]];
}

// bellypan areas that must stay solid under electrical parts (Pigeon 2 footprint)
function elecBellyKeep() returns array
{
    var k = [[PIGX, PIGY]];
    for (var sx in [-0.8, 0.8])
        for (var sy in [-0.8, 0.8])
            k = append(k, [PIGX + sx, PIGY + sy]);
    return k;
}

// ---- rear corner camera geometry (sy = +1 left, -1 right): yaw 35 deg out from straight back, 15 deg pitch up
function ll4RearFrame(sy is number) returns map
{
    const fh = vector(-cos(35 * degree), sy * sin(35 * degree), 0);
    const f = fh * cos(15 * degree) + vector(0, 0, sin(15 * degree));
    const u = -fh * sin(15 * degree) + vector(0, 0, cos(15 * degree));
    return { "Q" : P(-11.0, sy * 11.3, 14.3), "f" : f, "u" : u, "fh" : fh, "X" : cross(f, u) };
}

function ll4FootBolts(sy is number) returns array
{
    const F = ll4RearFrame(sy);
    var out = [];
    for (var x in [-1.0, 1.0])
    {
        const p = F.Q / inch - F.fh * 0.1 + F.X * x;
        out = append(out, [p[0], p[1]]);
    }
    return out;
}

// Limelight 4 on a printed wedge standing on the deck
function ll4Rear(context is Context, id is Id, k is string, sy is number) returns array
{
    const F = ll4RearFrame(sy);
    cotsAt(context, id + (k ~ "cam"), LL4::build, "EL-15 Limelight 4 (rear " ~ (sy > 0 ? "left" : "right") ~ ")", 0.33, LL4BACK, F.f, F.u, F.Q);
    // wedge profile in the vertical plane of the camera axis: s = distance behind Q, t = world z
    const qz = F.Q[2] / inch;
    const pts = [[-0.6, DECKT], [-0.6, qz - 0.84], [-0.25, qz - 0.94], [0.233, qz + 0.869], [0.475, qz + 0.804], [0.475, DECKT]];
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    const Xw = F.X;
    const o = vector(F.Q[0], F.Q[1], 0 * inch) + Xw * 1.45 * inch;
    prism(context, nextId(pid, c), coordSystem(o, -F.fh, -Xw), pts, 0, 2.9);
    var bl = [];
    for (var b in ll4FootBolts(sy))
    {
        sCyl(context, hid, c, P(b[0], b[1], DECKT - 0.1), P(b[0], b[1], DECKT + 0.6), D10 / 2);
        sCyl(context, hid, c, P(b[0], b[1], DECKT + 0.35), P(b[0], b[1], qz + 1.0), 0.22);      // counterbore for the bolt head
        bl = append(bl, [P(b[0], b[1], DECKT + 0.35), vector(0, 0, -1), 0.875, true]);
    }
    done(context, pid, hid, "EL-24 LL4 15 deg wedge mount (PETG)", C_PETG, "PETG (40% infill)");
    return bl;
}

// Limelight 4 on the turret (called from the turret feature, rotating group)
function ll4Turret(context is Context, rid is Id, ox is number) returns array
{
    const f = vector(cos(20 * degree), 0, sin(20 * degree));
    const u = vector(-sin(20 * degree), 0, cos(20 * degree));
    const Q = P(ox + 0.95, 5.575, 18.6);
    cotsAt(context, rid + "llcam", LL4::build, "TT-40 Limelight 4 (turret, 20 deg up)", 0.33, LL4BACK, f, u, Q);
    // L bracket: back plate behind the camera (local X -1.5 .. +1.875, reaches the side plate) + flange on the plate
    const pid = rid + "llbr";
    const hid = rid + "llbrh";
    const c = new box(0);
    const X = cross(f, u);                    // = -Y
    const o = Q - X * 1.525 * inch;           // outer end of the back plate (y 7.1)
    // back plate: in (X, u) plane, extruded backwards (-f) 0.125, reaching the side plate (y 3.7)
    prism(context, nextId(pid, c), coordSystem(o, X, -f), [[0, -0.9], [3.4, -0.9], [3.4, 0.9], [0, 0.9]], 0, 0.125);
    // flange on the outer face of the left side plate (y 3.7 .. 3.825)
    sBox(context, pid, c, P(ox + 0.62, 3.7, 18.05), P(ox + 3.95, 3.825, 19.15));
    for (var x in [ox + 1.4, ox + 3.6])
        sCyl(context, hid, c, P(x, 3.6, 18.6), P(x, 3.9, 18.6), D10 / 2);
    done(context, pid, hid, "TT-41 LL4 turret bracket 1/8 6061 (bent)", C_ALBK, "Aluminum 6061");
    return [[P(ox + 1.4, 3.825, 18.6), vector(0, -1, 0), 0.625, true], [P(ox + 3.6, 3.825, 18.6), vector(0, -1, 0), 0.625, true]];
}

annotation { "Feature Type Name" : "APEX 07 Electrical" }
export const apexElectrical = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        var bl = [];

        // ================= battery, sled, strap =================
        cotsAt(context, id + "batt", BATT::build, "EL-01 Battery 12 V 18 Ah SLA (MK ES17-12)", 12.9, [0, 0, 0], vector(1, 0, 0), vector(0, 0, 1), P(BATX, BATY, BATZ));
        {
            const pid = id + "sled";
            const hid = id + "sledh";
            const c = new box(0);
            const y0 = BATY - 3.85;
            const y1 = BATY + 3.85;
            sBox(context, pid, c, P(-FX, y0, BATZ - 0.125), P(-8.6, y1, BATZ));                 // base (on the rear rail)
            sBox(context, pid, c, P(-9.325, y0, BATZ), P(-9.2, y1, 6.3));                       // front stop
            for (var yy in [[y0, y0 + 0.125], [y1 - 0.125, y1]])
                sBox(context, pid, c, P(-12.5, yy[0], BATZ), P(-9.2, yy[1], 5.5));              // side guides
            for (var p in [[-13.0, -2.25], [-13.0, 4.25], [-8.9, -2.25], [-8.9, 4.25]])
                sCyl(context, hid, c, P(p[0], p[1], BATZ - 0.3), P(p[0], p[1], BATZ + 0.1), D10 / 2);
            for (var x in [-11.6, -10.3])                                                          // lightening slots under the battery
                for (var yy in [[-1.9, 0.0], [1.9, 3.8]])
                    sBox(context, hid, c, P(x - 0.45, yy[0], BATZ - 0.3), P(x + 0.45, yy[1], BATZ + 0.1));
            done(context, pid, hid, "EL-02 Battery sled 1/8 6061 (bent)", C_ALBK, "Aluminum 6061");
            for (var y in [-2.25, 4.25])
            {
                bl = append(bl, [P(-13.0, y, BATZ), vector(0, 0, -1), 0.375]);                  // into rail rivnuts
                standoff(context, id, "sleg" ~ (y > 0 ? "L" : "R"), P(-8.9, y, RZ0), P(-8.9, y, BATZ - 0.125));
                bl = append(bl, [P(-8.9, y, BATZ), vector(0, 0, -1), 0.375]);
                bl = append(bl, [P(-8.9, y, RZ0 - 0.125), vector(0, 0, 1), 0.375]);
            }
        }
        {
            const pid = id + "strap";
            const c = new box(0);
            const xs = BATX - 0.5;
            const ya = BATY - 3.61;
            const yb = BATY + 3.61;
            const zt = BATZ + 6.58;
            sBox(context, pid, c, P(xs, ya, zt), P(xs + 1.0, yb, zt + 0.06));
            sBox(context, pid, c, P(xs, ya, BATZ), P(xs + 1.0, ya + 0.06, zt + 0.06));
            sBox(context, pid, c, P(xs, yb - 0.06, BATZ), P(xs + 1.0, yb, zt + 0.06));
            sBox(context, pid, c, P(xs + 0.2, yb - 0.02, zt - 0.9), P(xs + 0.8, yb + 0.14, zt - 0.3));     // cam buckle
            done(context, pid, id + "straph", "EL-03 Battery strap 1 in nylon, cam buckle", C_BLK, "Nylon");
        }

        // SB50 battery connector (battery half on -Y, robot half on +Y), resting behind the strap
        {
            const pid = id + "sb";
            const c = new box(0);
            sBox(context, pid, c, P(-12.65, -1.3, 10.6), P(-11.75, 0.29, 11.45));
            sBox(context, pid, c, P(-12.6, 0.29, 10.65), P(-11.8, 0.31, 11.4));
            sBox(context, pid, c, P(-12.65, 0.31, 10.6), P(-11.75, 1.9, 11.45));
            done(context, pid, id + "sbh", "EL-04 Anderson SB50 connector pair, red (6 AWG)", C_RED, "Nylon");
        }

        // ================= main breaker: bent bracket hung in the rear deck pocket, over the battery =================
        {
            // shelf under the breaker, walls at the x ends, flanges bolted up under the deck; the y sides stay
            // open so the 6 AWG runs pass up through the pocket next to the breaker
            const pid = id + "bkb";
            const hid = id + "bkbh";
            const c = new box(0);
            sBox(context, pid, c, P(-13.0, -1.6, DECKZ - 0.35), P(-10.75, 1.6, DECKZ - 0.225));            // shelf
            sBox(context, pid, c, P(-13.0, -1.6, DECKZ - 0.35), P(-12.875, 1.6, DECKZ));                     // rear wall
            sBox(context, pid, c, P(-10.875, -1.6, DECKZ - 0.35), P(-10.75, 1.6, DECKZ));                    // front wall
            sBox(context, pid, c, P(-13.35, -1.6, DECKZ - 0.125), P(-12.875, 1.6, DECKZ));                   // rear flange
            sBox(context, pid, c, P(-10.875, -1.6, DECKZ - 0.125), P(-10.4, 1.6, DECKZ));                    // front flange
            for (var b in brkBolts())
                sCyl(context, hid, c, P(b[0], b[1], DECKZ - 0.3), P(b[0], b[1], DECKZ + 0.1), D10 / 2);
            for (var sx in [-0.485, 0.485])
                for (var sy in [-1.1, 1.1])
                    sCyl(context, hid, c, P(BRKX + sx, sy, DECKZ - 0.5), P(BRKX + sx, sy, DECKZ), D10 / 2);
            sBox(context, hid, c, P(BRKX - 0.75, -0.7, DECKZ - 0.5), P(BRKX + 0.75, 0.7, DECKZ));           // trip-pin window
            done(context, pid, hid, "EL-06 Breaker bracket 1/8 6061 (bent)", C_ALBK, "Aluminum 6061");
            for (var b in brkBolts())
                bl = append(bl, [P(b[0], b[1], DECKT), vector(0, 0, -1), 0.625, true]);
            for (var sx in [-0.485, 0.485])
                for (var sy in [-1.1, 1.1])
                    bl = append(bl, [P(BRKX + sx, sy, DECKZ - 0.225 + 1.09), vector(0, 0, -1), 1.5, true]);
        }
        cotsAt(context, id + "brkr", BRKR::build, "EL-05 Main breaker 120 A (Eaton Bussmann CB185-120)", 0.25, [0, 0, -0.89], vector(0, 1, 0), vector(0, 0, 1), P(BRKX, 0, DECKZ - 0.225));

        // ================= electronics boards (hung under the deck, open sides) =================
        for (var sy in [1, -1])
        {
            const s = sy > 0 ? "L" : "R";
            // board outline, inner front corner cut around the bowl (r 8.6 about the rotor axis)
            const yi = 7.85;
            const xc = ROTX - sqrt(8.6 * 8.6 - yi * yi);
            const yc = sqrt(8.6 * 8.6 - (ROTX + 1.35) * (ROTX + 1.35));
            var o = [[-12.375, sy * yi], [xc, sy * yi]];
            const a0 = atan2(yi, xc - ROTX) / degree;
            const a1 = atan2(yc, -1.35 - ROTX) / degree;
            for (var i = 1; i < 6; i += 1)
            {
                const a = (a0 + (a1 - a0) * i / 6) * degree;
                o = append(o, [ROTX + 8.6 * cos(a), sy * 8.6 * sin(a)]);
            }
            o = concat(o, [[-1.35, sy * yc], [-1.35, sy * 13.75], [-12.375, sy * 13.75]]);
            if (sy < 0)
                o = rev(o);
            var holes = [[-1.7, sy * 12.4, D10], [-12.11, sy * 8.1, D10], [-12.11, sy * 13.45, D10]];
            const pk = sy > 0 ? [rrect(-3.0, 11.2, -2.0, 13.4, 0.25)] : [];         // harness pass-through under the PDH outer row
            plate(context, id, "brd" ~ s, "EL-2" ~ (sy > 0 ? 0 : 1) ~ " Electronics board " ~ s ~ " 3/16 polycarb", csXY(TRAYZ - TRAYT), o, TRAYT,
                { "r" : 0.3, "holes" : holes, "pockets" : pk, "rp" : 0.25 }, C_PCS, "Polycarbonate");
            // hanger: top flange (bolted up into the deck), web, bottom flange under the board
            {
                const pid = id + ("hng" ~ s);
                const hid = id + ("hngh" ~ s);
                const c = new box(0);
                const ya = sy > 0 ? 7.95 : -12.7;
                const yb = sy > 0 ? 12.7 : -7.95;
                const yc2 = sy > 0 ? 13.6 : -13.6;
                sBox(context, pid, c, P(-13.25, ya, DECKZ - 0.125), P(-12.375, yb, DECKZ));
                sBox(context, pid, c, P(-12.5, sy * 7.95, TRAYZ - TRAYT - 0.125), P(-12.375, yc2, DECKZ));
                sBox(context, pid, c, P(-12.5, sy * 7.95, TRAYZ - TRAYT - 0.125), P(-11.85, yc2, TRAYZ - TRAYT));
                for (var y in [8.6, 12.1])
                    sCyl(context, hid, c, P(-12.9, sy * y, DECKZ - 0.3), P(-12.9, sy * y, DECKZ + 0.1), D10 / 2);
                for (var y in [8.1, 13.45])
                    sCyl(context, hid, c, P(-12.11, sy * y, TRAYZ - 0.6), P(-12.11, sy * y, TRAYZ), D10 / 2);
                for (var zz in [[TRAYZ + 0.35, 9.55], [9.85, 12.3]])                       // lightening windows in the web
                    sBox(context, hid, c, P(-12.6, sy > 0 ? 8.55 : -12.1, zz[0]), P(-12.3, sy > 0 ? 12.1 : -8.55, zz[1]));
                done(context, pid, hid, "EL-22 Board hanger " ~ s ~ " 1/8 6061 (bent)", C_ALBK, "Aluminum 6061");
            }
            for (var y in [8.6, 12.1])
                bl = append(bl, [P(-12.9, sy * y, DECKT), vector(0, 0, -1), 0.625, true]);
            for (var y in [8.1, 13.45])
                bl = append(bl, [P(-12.11, sy * y, TRAYZ), vector(0, 0, -1), 0.625, true]);
            spacer(context, id, "post" ~ s, "EL-23 Board post 1/2 OD x 5.75 (10-32 tapped)", P(-1.7, sy * 12.4, TRAYZ), P(-1.7, sy * 12.4, DECKZ), 0.25, 0.08, C_AL);
            bl = append(bl, [P(-1.7, sy * 12.4, DECKT), vector(0, 0, -1), 0.5]);
            bl = append(bl, [P(-1.7, sy * 12.4, TRAYZ - TRAYT), vector(0, 0, 1), 0.5]);
        }
        // left board: PDH (main input end toward the breaker; turret harness grommet right above its front end)
        cotsAt(context, id + "pdh", PDH::build, "EL-07 REV Power Distribution Hub", 1.1, [0, 0, 0], vector(1, 0, 0), vector(0, 0, 1), P(PDHX, PDHY, TRAYZ));
        // right board: roboRIO 2 (USB / Ethernet edge facing the open side), CANivore next to its USB port, MPM
        cotsAt(context, id + "rio", RIO2::build, "EL-08 NI roboRIO 2.0", 0.75, [0, 0.38, 0], vector(0, 0, -1), vector(0, -1, 0), P(RIOX, RIOY, TRAYZ));
        cotsAt(context, id + "caniv", CANIV::build, "EL-09 CTRE CANivore (drivetrain CAN FD)", 0.07, [0, -0.174, 0.04], vector(0, 0, -1), vector(0, 1, 0), P(-2.25, -12.17, TRAYZ));
        cotsAt(context, id + "mpm", MPM::build, "EL-10 REV Mini Power Module", 0.12, [0, 0, 0], vector(1, 0, 0), vector(0, 0, 1), P(-4.25, -8.95, TRAYZ));

        // ================= radio, RSL, IMU =================
        {
            var holes = [];
            for (var p in radioMountHoles())
                holes = append(holes, [p[0], p[1], D10]);
            plate(context, id, "rmnt", "EL-25 Radio mount (PETG)", csXY(DECKT), [[-9.35, 9.35], [-3.85, 9.35], [-3.85, 11.85], [-9.35, 11.85]], 0.25,
                { "r" : 0.3, "holes" : holes, "pockets" : [rrect(-8.4, 9.9, -4.8, 11.3, 0.3)] }, C_PETG, "PETG (40% infill)");
            for (var h in holes)
                bl = append(bl, [P(h[0], h[1], DECKT + 0.25), vector(0, 0, -1), 0.75, true]);
        }
        {
            var holes = [];
            for (var p in switchMountHoles())
                holes = append(holes, [p[0], p[1], D10]);
            plate(context, id, "smnt", "EL-26 Switch mount (PETG)", csXY(DECKT), [[-8.95, -12.5], [-4.65, -12.5], [-4.65, -9.3], [-8.95, -9.3]], 0.25,
                { "r" : 0.3, "holes" : holes }, C_PETG, "PETG (40% infill)");
            for (var h in holes)
                bl = append(bl, [P(h[0], h[1], DECKT + 0.25), vector(0, 0, -1), 0.75, true]);
        }
        cotsAt(context, id + "sw", SWCH::build, "EL-12 Network switch, 5 port 12 V", 0.25, [0, 0, -0.866], vector(0, 1, 0), vector(0, 0, 1), P(-6.75, -10.9, DECKT + 0.25));
        // radio long side along X so it clears the wrap ring (r >= 9.37 about the turret axis)
        cotsAt(context, id + "vh", VH109::build, "EL-11 Vivid-Hosting VH-109 radio", 0.55, [0.043, 0, 0], vector(0, 0, -1), vector(0, 1, 0), P(-6.555, 10.6, DECKT + 0.25));
        cotsAt(context, id + "rsl", RSL::build, "EL-14 Robot Signal Light (Allen-Bradley 855PB)", 0.25, [0, 0, 0], vector(0, 1, 0), vector(0, 0, 1), P(RSLX, RSLY, DECKT));
        cotsAt(context, id + "pig", PIG2::build, "EL-13 CTRE Pigeon 2.0 IMU", 0.06, [0, 0, 0], vector(0, 1, 0), vector(0, 0, 1), P(PIGX, PIGY, RZ0));

        // ================= cameras and ball sensors =================
        bl = concat(bl, ll4Rear(context, id, "llL", 1));
        bl = concat(bl, ll4Rear(context, id, "llR", -1));
        cotsAt(context, id + "cr1", CANR::build, "EL-16 CTRE CANrange (uptake ball)", 0.04, [0, 0.71, 0.045], vector(-1, 0, 0), vector(0, 0, 1), P(TURX - UPP + 0.5, UPY + 0.47, 9.85));
        // hopper-full CANrange looks down from the left liner, on a printed bracket (bolts through the liner)
        cotsAt(context, id + "cr2", CANR::build, "EL-16 CTRE CANrange (hopper full)", 0.04, [0, 0.71, 0.045], vector(0, 0, 1), vector(1, 0, 0), P(1.5, 12.0, 20.0));
        {
            const pid = id + "crb";
            const c = new box(0);
            sBox(context, pid, c, P(0.8, 12.68, 19.35), P(2.2, 12.875, 19.95));
            done(context, pid, id + "crbh", "EL-27 CANrange bracket (PETG)", C_PETG, "PETG (40% infill)");
            for (var x in [1.1, 1.9])
                bl = append(bl, [P(x, 13.0, 19.65), vector(0, -1, 0), 0.25]);
        }

        bolts(context, id, "hw", bl);
    });
