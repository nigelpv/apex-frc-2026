
// =========================================================================
// 03 SERIALIZER ("dye rotor"): 16.2 in rotor turning on a dead axle above the
// electronics bay. Five pockets on a 5.2 in circle carry FUEL to the rear
// (180 deg) pocket, which sits on the turret axis under the uptake.
//   rotor stack: 160T 10DP 7075 gear plate (spoke pocketed) + 1/8 polycarb floor
//   + 5 printed fins + printed hub cone; FR8 flanged bearings on a 1/2 in axle.
//   drive pod at -90 deg: X60 (shaft down) -> 18T:18T HTD belt -> 1/2 hex
//   jackshaft -> 12T 10DP pinion  => 13.33:1 at the rotor.
//   fixed: rolled 1/16 polycarb bowl on four posts, UHMW thrust pucks.
// =========================================================================
annotation { "Feature Type Name" : "APEX 03 Serializer" }
export const apexSerializer = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Rotor angle" }
        isAngle(definition.rotor, ANGLE_360_ZERO_DEFAULT_BOUNDS);
    }
    {
        const rid = id + "rot";             // everything that turns with the rotor
        const cx = ROTX;

        // ---- RS-01 gear plate: 160T 10DP, 1/4 7075, ten spoke pockets, FR8 bore
        {
            var dec = [];
            for (var i = 0; i < 10; i += 1)
                dec = append(dec, [cx + 6.7 * cos(36 * i * degree), 6.7 * sin(36 * i * degree)]);
            var holes = [[cx, 0, 1.125]];
            for (var i = 0; i < 10; i += 1)
                holes = append(holes, [cx + 7.2 * cos((18 + 36 * i) * degree), 7.2 * sin((18 + 36 * i) * degree), D10]);
            for (var i = 0; i < 5; i += 1)
                for (var r in [3.0, 6.3])
                    holes = append(holes, [cx + r * cos(72 * i * degree), r * sin(72 * i * degree), D10]);
            plate(context, rid, "gp", "RS-01 Rotor gear plate 160T 10DP 1/4 7075 (spoked)", csXY(ROTZ), gearPts(cx, 0, 160, 10), 0.25,
                { "r" : 0, "holes" : holes, "pockets" : fanPockets(dec, cx, 0, 1.5, 0.5), "rp" : 0.3 }, C_GOLD, "Aluminum 7075");
        }

        // ---- RS-02 floor: 1/8 polycarb disc
        {
            var holes = [];
            for (var i = 0; i < 10; i += 1)
                holes = append(holes, [cx + 7.2 * cos((18 + 36 * i) * degree), 7.2 * sin((18 + 36 * i) * degree), D10]);
            for (var i = 0; i < 5; i += 1)
                for (var r in [3.0, 6.3])
                    holes = append(holes, [cx + r * cos(72 * i * degree), r * sin(72 * i * degree), D10]);
            plate(context, rid, "fl", "RS-02 Rotor floor 1/8 polycarb", csXY(ROTZ + 0.25), circPath(cx, 0, 7.75), 0.125,
                { "r" : 0, "holes" : concat([[cx, 0, 1.5]], holes) }, C_PCX, "Polycarbonate");
        }

        // ---- RS-03 fins (printed PETG, radial, between the pockets)
        for (var i = 0; i < 5; i += 1)
        {
            const a = 72 * i * degree;
            const u = vector(cos(a), sin(a), 0);
            const n = vector(sin(a), -cos(a), 0);
            const cs = coordSystem(P(cx, 0, 0), u, n);
            plate(context, rid, "fin" ~ i, "RS-03 Rotor fin (PETG print, 40% gyroid)", cs,
                [[1.95, RFZ], [7.55, RFZ], [7.55, FINZ - 0.3], [7.25, FINZ], [2.3, FINZ], [1.95, FINZ - 0.25]], 0.25,
                { "r" : 0.08, "w0" : -0.125 }, C_PETG, "PETG (40% infill)");
        }

        // ---- RS-04 hub cone (printed): deflects FUEL into the pockets, houses the upper bearing
        turned(context, rid, "cone", "RS-04 Rotor hub cone (PETG print)", P(cx, 0, RFZ), vector(0, 0, 1),
            [[0, 0.6], [0, 1.9], [0.12, 1.9], [0.8, 1.6], [1.4, 1.22], [1.9, 0.82], [2.25, 0.42], [2.32, 0.0], [1.7, 0.0], [1.7, 0.3],
                [1.5625, 0.3], [1.5625, 0.5625], [1.31, 0.5625], [1.31, 0.66], [1.24, 0.66], [1.24, 0.6]], 0, 0, C_PETG, "PETG (40% infill)");     // upper bearing seat + flange counterbore

        // ---- rotor bearings + spacer (turn with the rotor)
        fbearing(context, rid, "brgL", P(cx, 0, ROTZ + 0.25 - 0.15625), vector(0, 0, -1), 1.125, 0.3125, 0, 0.25);
        fbearing(context, rid, "brgU", P(cx, 0, RFZ + 1.40625), vector(0, 0, -1), 1.125, 0.3125, 0, 0.25);     // flange down, on the spacer
        spacer(context, rid, "spc", "RS-05 Bearing spacer 3/4 OD x 1/2 ID", P(cx, 0, ROTZ + 0.25), P(cx, 0, RFZ + 1.25), 0.375, 0.26, C_AL);

        // ---- fin / floor hardware: BHCS up through the gear plate into heat-set inserts
        {
            var bl = [];
            for (var i = 0; i < 5; i += 1)
                for (var r in [3.0, 6.3])
                    bl = append(bl, [P(cx + r * cos(72 * i * degree), r * sin(72 * i * degree), ROTZ), vector(0, 0, 1), 0.75]);
            for (var i = 0; i < 10; i += 1)
                bl = append(bl, [P(cx + 7.2 * cos((18 + 36 * i) * degree), 7.2 * sin((18 + 36 * i) * degree), ROTZ), vector(0, 0, 1), 0.625, true]);
            bolts(context, rid, "hw", bl);
        }
        spinZ(context, id + "spin", qCreatedBy(rid, EntityType.BODY), cx, 0, definition.rotor);

        // ---- fixed: dead axle, axle block, collar
        sCylPart(context, id, "axle", "RS-06 Rotor dead axle 1/2 in 4140 (5/16-18 tapped ends)", P(cx, 0, 1.75), P(cx, 0, RFZ + 1.625), 0.25, C_ST, "Steel");
        {
            const pid = id + "blk";
            const hid = id + "blkh";
            const c = new box(0);
            prismR(context, nextId(pid, c), csXY(1.75), [[cx - 0.75, -0.75], [cx + 0.75, -0.75], [cx + 0.75, 0.75], [cx - 0.75, 0.75]], 0.12, 0, 1.25);
            sCyl(context, hid, c, P(cx, 0, 1.7), P(cx, 0, 3.1), 0.25);
            for (var sx in [-0.5, 0.5])
                sCyl(context, hid, c, P(cx + sx, 0, 1.7), P(cx + sx, 0, 3.1), D10 / 2);
            done(context, pid, hid, "RS-07 Axle block 1.5 x 1.5 x 1.25 6061", C_AL, "Aluminum 6061");
        }
        collar(context, id, "col", P(cx, 0, ROTZ - 0.25), vector(0, 0, 1), 1.125, 0.375, 0, 0.25);

        // ---- thrust pucks under the gear rim (UHMW on standoffs)
        var bl = [];
        for (var a in [60, 180, 300])
        {
            const x = cx + 7.3 * cos(a * degree);
            const y = 7.3 * sin(a * degree);
            standoff(context, id, "pso" ~ a, P(x, y, 1.75), P(x, y, 3.95));
            sCylPart(context, id, "puck" ~ a, "RS-08 Thrust puck 1 in UHMW", P(x, y, 3.95), P(x, y, ROTZ - 0.03), 0.5, C_UHMW, "UHMW");
            bl = append(bl, [P(x, y, BPZ), vector(0, 0, 1), 0.5]);
        }
        for (var sx in [-0.5, 0.5])
            bl = append(bl, [P(cx + sx, 0, BPZ), vector(0, 0, 1), 1.25]);

        // ---- RS-10 bowl wall: rolled 1/16 polycarb, open at 180 deg for the uptake belts
        {
            const pid = id + "bowl";
            const hid = id + "bowlh";
            const c = new box(0);
            sTube(context, pid, hid, c, P(cx, 0, BOWLZ0), P(cx, 0, BOWLZ1), BOWLR + 0.0625, BOWLR);
            sBox(context, hid, c, P(cx - BOWLR - 1, -1.15, BOWLZ0 - 0.1), P(cx - BOWLR + 0.8, 1.15, BOWLZ1 + 0.1));
            done(context, pid, hid, "RS-10 Bowl wall 1/16 polycarb (rolled, riveted butt strap)", C_PCX, "Polycarbonate");
        }
        // bowl posts + printed clips
        for (var a in BOWLPOSTS)
        {
            const x = cx + 8.7 * cos(a * degree);
            const y = 8.7 * sin(a * degree);
            spacer(context, id, "post" ~ a, "RS-11 Bowl post 1/2 OD aluminium standoff", P(x, y, 1.75), P(x, y, BOWLZ0 - 0.1), 0.25, 0.08, C_AL);
            {
                const pid = id + ("clip" ~ a);
                const hid = id + ("cliph" ~ a);
                const c = new box(0);
                const u = vector(cos(a * degree), sin(a * degree), 0);
                const cs = coordSystem(P(cx, 0, BOWLZ0 - 0.1), u, vector(0, 0, 1));
                prismR(context, nextId(pid, c), cs, [[8.15, -0.45], [9.1, -0.45], [9.1, 0.45], [8.15, 0.45]], 0.1, 0, 1.0);
                revolveSolid(context, nextId(hid, c), P(cx, 0, 0), vector(0, 0, 1),
                    [[BOWLZ0 - 0.01, BOWLR - 0.01], [BOWLZ0 + 1.2, BOWLR - 0.01], [BOWLZ0 + 1.2, BOWLR + 0.075], [BOWLZ0 - 0.01, BOWLR + 0.075]]);
                sCyl(context, hid, c, P(x, y, BOWLZ0 - 0.2), P(x, y, BOWLZ0 + 1.0), 0.08);
                done(context, pid, hid, "RS-12 Bowl clip (PETG print)", C_PETG, "PETG (40% infill)");
            }
            bl = append(bl, [P(x, y, BPZ), vector(0, 0, 1), 0.5]);
        }

        // ---- RS-20 drive pod at -90 deg
        {
            const jy = -8.6;            // jackshaft (pinion centre distance 8.6)
            const my = -10.3;           // motor axis
            const ox = cx;
            const outline = [[ox - 1.6, -11.6], [ox + 1.6, -11.6], [ox + 1.6, -8.05], [ox + 0.9, -7.95], [ox - 0.9, -7.95], [ox - 1.6, -8.05]];
            var hb = [[ox, jy, 1.125]];
            var ht = [[ox, jy, 1.125], [ox, my, 1.0]];
            for (var q in [[-1.2, -11.1], [1.2, -11.1], [-1.2, -8.45], [1.2, -8.45]])
            {
                hb = append(hb, [ox + q[0], q[1], D10]);
                ht = append(ht, [ox + q[0], q[1], D10]);
            }
            for (var q in [[-1.2, -9.75], [1.2, -9.75]])
                hb = append(hb, [ox + q[0], q[1], D10]);
            for (var a in [30, 150, 210, 330])          // X60 face pattern: 2.0 in circle, 30 deg steps
                ht = append(ht, [ox + cos(a * degree), my + sin(a * degree), D10]);
            plate(context, id, "podB", "RS-21 Drive pod lower plate 1/4 6061", csXY(3.85), outline, 0.25, { "r" : 0.25, "holes" : hb }, C_ALBK, "Aluminum 6061");
            plate(context, id, "podT", "RS-22 Drive pod upper plate 1/4 6061 (motor mount)", csXY(5.5), outline, 0.25, { "r" : 0.25, "holes" : ht }, C_ALBK, "Aluminum 6061");
            var si = 0;
            for (var q in [[-1.2, -11.1], [1.2, -11.1], [-1.2, -8.45], [1.2, -8.45]])
            {
                si += 1;
                standoff(context, id, "pst" ~ si, P(ox + q[0], q[1], 4.1), P(ox + q[0], q[1], 5.5));
                bl = append(bl, [P(ox + q[0], q[1], 5.75), vector(0, 0, -1), 0.375]);
                bl = append(bl, [P(ox + q[0], q[1], 3.85), vector(0, 0, 1), 0.375]);
            }
            for (var q in [[-1.2, -9.75], [1.2, -9.75]])
            {
                si += 1;
                standoff(context, id, "pleg" ~ si, P(ox + q[0], q[1], 1.75), P(ox + q[0], q[1], 3.85));
                bl = append(bl, [P(ox + q[0], q[1], 4.1), vector(0, 0, -1), 0.375]);
                bl = append(bl, [P(ox + q[0], q[1], BPZ), vector(0, 0, 1), 0.375]);
            }
            for (var a in [30, 150, 210, 330])
                bl = append(bl, [P(ox + cos(a * degree), my + sin(a * degree), 5.5), vector(0, 0, 1), 0.5]);
            kraken(context, id + "mot", P(ox, my, 5.75), vector(0, 0, -1), true);
            hexShaft(context, id, "js", "RS-23 Pod jackshaft 1/2 hex 2.1 in", P(ox, jy, 3.72), P(ox, jy, 5.86), 0.5, C_ST, "Steel");
            fbearing(context, id, "jbL", P(ox, jy, 3.85 + 0.25 - 0.15625), vector(0, 0, -1), 1.125, 0.3125, 0.5, 0);
            fbearing(context, id, "jbU", P(ox, jy, 5.75 - 0.15625), vector(0, 0, -1), 1.125, 0.3125, 0.5, 0);     // flange under the plate (motor can above)
            // a rotor tooth points at the pinion at rotor angle 0: put a pinion gap there and keep the mesh in phase
            gearPartR(context, id, "pin", "RS-24 Rotor pinion 12T 10DP steel (1/2 hex)", P(ox, jy, 4.4), 12, 10, 0.45, 0.5, 15 - definition.rotor / degree * 160 / 12);
            htd(context, id, "jp", P(ox, jy, 5.1), vector(0, 0, 1), 18, 0.354, 0.5);
            htd(context, id, "mp", P(ox, my, 5.1), vector(0, 0, 1), 18, 0.354, 0.315);
            beltLoop(context, id, "belt", "RS-25 Pod belt 9 mm HTD 5 mm, 70T", P(ox, jy, 5.1), 0.564, P(ox, my, 5.1), 0.564, vector(0, 0, 1), 0.354);
        }
        boltsIn(context, id, "hw", bl, qSubtraction(qCreatedBy(id, EntityType.BODY), qCreatedBy(rid, EntityType.BODY)));
    });

// plain cylinder part (axles, pins, pucks: never drilled)
function sCylPart(context is Context, id is Id, k is string, name is string, a is Vector, b is Vector, r is number, col is Color, m is string)
{
    const pid = id + k;
    const c = new box(0);
    sCyl(context, pid, c, a, b, r);
    done(context, pid, pid + "h", name, col, m);
    nodrill(context, qCreatedBy(pid, EntityType.BODY));
}

// horizontal spur gear (axis +Z) with hex bore, bottom face at p
function gearPart(context is Context, id is Id, k is string, name is string, p is Vector, teeth is number, dp is number, t is number, af is number)
{
    gearPartR(context, id, k, name, p, teeth, dp, t, af, 0);
}

// ... with tooth 0 at rot degrees (CCW from +X), for meshing phase
function gearPartR(context is Context, id is Id, k is string, name is string, p is Vector, teeth is number, dp is number, t is number, af is number, rot is number)
{
    const pid = id + k;
    const hid = id + (k ~ "h");
    const c = new box(0);
    const cs = coordSystem(p, vector(1, 0, 0), vector(0, 0, 1));
    sPoly(context, pid, c, cs, gearPtsR(0, 0, teeth, dp, rot), 0, t);
    sPoly(context, hid, c, cs, hexPts(0, 0, af), -0.1, t + 0.1);
    done(context, pid, hid, name, C_ST, "Steel");
    nodrill(context, qCreatedBy(pid, EntityType.BODY));
}
