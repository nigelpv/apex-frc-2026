
// =========================================================================
// 05 TURRET SHOOTER. Fixed: 1/2 in bearing housing on the deck, Kaydon
// KC090XP0 X-contact bearing (9.0 x 10.0 x 0.5), yaw drive (X44 + 7:1
// MAXPlanetary -> 12T pinion on the 120T 10DP turret plate = 70:1), dual
// CANcoder vernier pods (19T / 21T, absolute over 598 deg), cable-wrap ring.
// Rotating: turret plate, 1/4 truss side plates, 3x 4 in flywheel wheels + 2
// steel discs on 1/2 hex, 2x X60 1:1 (18T:18T HTD, outside the plates), back
// guide, PTFE-lined polycarb hood on 6061 ribs pivoting behind the flywheel
// (X44 + 25:1 -> 16T:45T 20DP sector), turret Limelight 4, cable drum.
// Cable wrap: the turret harness (6x 12 AWG, 2x 18 AWG, CAN, Cat6 zip-tied to
// a polycarb backing strip, 0.6 x 0.3 in) is a horizontal service loop in an
// annular channel above the gears: from its fixed end on the ring wall (A)
// it runs CCW along the wall, makes a U-turn and runs back along the drum to
// the moving end (B). The U-turn travels at 0.46x the turret angle, so the
// loop covers +/-190 deg with 10 deg of margin at each end.
// Every gear that meshes with the turret plate sits on a plate tooth gap and
// is phased for the configured turret angle (and the hood pinion for the hood
// lift), so the mesh stays clean in every configuration.
// Local frame (turret angle 0): +X launch direction.
// =========================================================================
annotation { "Feature Type Name" : "APEX 05 Turret" }
export const apexTurret = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Turret angle (0 = launch toward the intake)" }
        isAngle(definition.turret, ANGLE_360_ZERO_DEFAULT_BOUNDS);
        annotation { "Name" : "Hood lift (0 = 45 deg launch, stowed for the TRENCH)" }
        isAngle(definition.hood, { (degree) : [0, 0, 25] } as AngleBoundSpec);
    }
    {
        const ox = TURX;
        const th = definition.turret / degree;
        var bl = [];        // fixed hardware
        var br = [];        // rotating hardware

        // ================= fixed =================
        // TT-01 bearing housing (1/2 in 6061 ring, counterbored seat for the outer race)
        {
            const pid = id + "hsg";
            const hid = id + "hsgh";
            const c = new box(0);
            revolveSolid(context, nextId(pid, c), P(ox, 0, 0), vector(0, 0, 1),
                [[DECKT, 4.85], [DECKT, 5.75], [DECKT + 0.5, 5.75], [DECKT + 0.5, 5.0], [BRGZ0, 5.0], [BRGZ0, 4.85]]);
            for (var i = 0; i < 8; i += 1)
            {
                const a = (22.5 + 45 * i) * degree;
                sCyl(context, hid, c, P(ox + 5.35 * cos(a), 5.35 * sin(a), DECKT - 0.1), P(ox + 5.35 * cos(a), 5.35 * sin(a), DECKT + 0.6), D10 / 2);
                bl = append(bl, [P(ox + 5.35 * cos(a), 5.35 * sin(a), DECKT + 0.5), vector(0, 0, -1), 1.0, true]);
            }
            done(context, pid, hid, "TT-01 Turret bearing housing 1/2 in 6061", C_ALBK, "Aluminum 6061");
        }
        turned(context, id, "brgO", "TT-02 Kaydon KC090XP0 bearing, outer race", P(ox, 0, 0), vector(0, 0, 1),
            [[BRGZ0, 4.76], [BRGZ0, 4.98], [BRGZ0 + 0.03, 5.0], [BRGZ1 - 0.03, 5.0], [BRGZ1, 4.98], [BRGZ1, 4.76]], 0, 0, C_ST, "Steel");
        nodrill(context, qCreatedBy(id + "brgO", EntityType.BODY));

        // yaw drive: X44 + 7:1 MAXPlanetary under the deck, 12T pinion above (on a plate tooth gap)
        const ya = YAWA * degree;
        const yx = ox + 6.61 * cos(ya);
        const yy = 6.61 * sin(ya);
        kraken(context, id + "ymot", P(yx, yy, 11.0), vector(0, 0, 1), false);
        turned(context, id, "ygb", "TT-03 REV MAXPlanetary 7:1 (1 stage)", P(yx, yy, 11.0), vector(0, 0, 1),
            [[0, 0.4], [0, 0.88], [1.55, 0.88], [1.55, 1.02], [1.75, 1.02], [1.75, 0.3], [0.25, 0.3], [0.25, 0.4]], 0, 0, C_BLK, "Aluminum 6061");
        hexShaft(context, id, "ysh", "TT-04 Yaw output shaft 1/2 hex", P(yx, yy, 12.75), P(yx, yy, 13.86), 0.5, C_ST, "Steel");
        {
            // pinion tooth on the line of centres (into the plate gap); turns -120/12 x the turret
            const base = (YAWA - 180) - floor((YAWA - 180) / 30) * 30;
            gearPartR(context, id, "ypin", "TT-05 Yaw pinion 12T 10DP steel", P(yx, yy, 13.575), 12, 10, 0.3, 0.5, base - th * 120 / 12);
        }

        // dual CANcoder vernier pods (19T, 21T on the 120T plate)
        var ei = 0;
        for (var q in [[ENC19A, 19], [ENC21A, 21]])
        {
            ei += 1;
            const a = q[0] * degree;
            const rr = 6.0 + q[1] / 20 + 0.01;
            const ex = ox + rr * cos(a);
            const ey = rr * sin(a);
            {
                const pid = id + ("epod" ~ ei);
                const c = new box(0);
                const hid = id + ("epodh" ~ ei);
                prismR(context, nextId(pid, c), csXY(DECKT), [[ex - 1.3, ey - 0.8], [ex + 1.3, ey - 0.8], [ex + 1.3, ey + 0.8], [ex - 1.3, ey + 0.8]], 0.2, 0, 0.12);
                for (var sx in [-1.08, 1.08])
                    sCyl(context, hid, c, P(ex + sx, ey, DECKT - 0.1), P(ex + sx, ey, DECKT + 0.3), D10 / 2);
                done(context, pid, hid, "TT-06 Encoder pod base (PETG print)", C_PETG, "PETG (40% infill)");
            }
            {
                const pid = id + ("cc" ~ ei);
                const c = new box(0);
                prismR(context, nextId(pid, c), csXY(DECKT + 0.12), [[ex - 0.85, ey - 0.69], [ex + 0.85, ey - 0.69], [ex + 0.85, ey + 0.69], [ex - 0.85, ey + 0.69]], 0.15, 0, 0.42);
                sCyl(context, pid, c, P(ex, ey, DECKT + 0.54), P(ex, ey, DECKT + 0.6), 0.4);
                done(context, pid, pid + "h", "TT-07 CTRE CANcoder (turret vernier " ~ q[1] ~ "T)", C_BLK, "Nylon");
                nodrill(context, qCreatedBy(pid, EntityType.BODY));
            }
            sCylPart(context, id, "esh" ~ ei, "TT-08 Encoder shaft 1/4 in (magnet end)", P(ex, ey, DECKT + 0.6), P(ex, ey, 13.87), 0.125, C_ST, "Steel");
            const pitch = 360 / q[1];
            const base = (q[0] - 180) - floor((q[0] - 180) / pitch) * pitch;
            gearPartR(context, id, "eg" ~ ei, "TT-09 Encoder gear " ~ q[1] ~ "T 10DP (Delrin)", P(ex, ey, BRGZ1), q[1], 10, 0.25, 0.25, base - th * 120 / q[1]);
            bl = append(bl, [P(ex - 1.08, ey, DECKT + 0.12), vector(0, 0, -1), 0.625, true]);
            bl = append(bl, [P(ex + 1.08, ey, DECKT + 0.12), vector(0, 0, -1), 0.625, true]);
        }

        // TT-10 cable-wrap ring (fixed): printed floor + wall above the gears, on six posts from the deck.
        // The A-side harness comes in through the wall at WRAPAS.
        {
            const pid = id + "ring";
            const hid = id + "ringh";
            const c = new box(0);
            revolveSolid(context, nextId(pid, c), P(ox, 0, 0), vector(0, 0, 1),
                [[13.85, RINGRI], [13.85, FENR], [14.7, FENR], [14.7, RINGRW], [14.0, RINGRW], [14.0, RINGRI]]);
            for (var p in ringPosts())
                sCyl(context, hid, c, P(p[0], p[1], 13.8), P(p[0], p[1], 14.05), 0.0795);       // heat-set inserts
            const aa = WRAPAS * degree;
            sCyl(context, hid, c, P(ox + 8.6 * cos(aa), 8.6 * sin(aa), WRAPZ), P(ox + 9.3 * cos(aa), 9.3 * sin(aa), WRAPZ), 0.2);
            done(context, pid, hid, "TT-10 Cable-wrap ring (PETG print, 4 segments)", C_PETG, "PETG (40% infill)");
        }
        var pi = 0;
        for (var p in ringPosts())
        {
            pi += 1;
            spacer(context, id, "rpost" ~ pi, "TT-62 Ring post 3/8 OD x 0.85", P(p[0], p[1], DECKT), P(p[0], p[1], 13.85), 0.1875, 0.1, C_AL);
            bl = append(bl, [P(p[0], p[1], DECKZ), vector(0, 0, 1), 1.25]);      // up through the deck and post into the ring floor
        }

        bolts(context, id, "hw", bl);

        // TT-11 service loop, drawn for the configured turret angle
        {
            const rho = (WRAPRO - WRAPRI) / 2;
            const ru = (WRAPRO + WRAPRI) / 2;
            const aA = WRAPA;
            const aB = WRAPB0 + th;
            const aU = ((WRAPLEN - PI * rho) * 180 / PI + WRAPRO * aA + WRAPRI * aB) / (WRAPRO + WRAPRI);
            const pid = id + "loop";
            const c = new box(0);
            const z0 = WRAPZ - 0.3;
            const z1 = WRAPZ + 0.3;
            revolveArcZ(context, nextId(pid, c), ox, 0, [[z0, WRAPRO - 0.15], [z1, WRAPRO - 0.15], [z1, WRAPRO + 0.15], [z0, WRAPRO + 0.15]], aA, aU);
            revolveArcZ(context, nextId(pid, c), ox, 0, [[z0, WRAPRI - 0.15], [z1, WRAPRI - 0.15], [z1, WRAPRI + 0.15], [z0, WRAPRI + 0.15]], aB, aU);
            revolveArcZ(context, nextId(pid, c), ox + ru * cos(aU * degree), ru * sin(aU * degree),
                [[z0, rho - 0.15], [z1, rho - 0.15], [z1, rho + 0.15], [z0, rho + 0.15]], aU, aU + 180);
            done(context, pid, pid + "h", "TT-11 Turret service loop (6x 12 AWG, 2x 18 AWG, CAN, Cat6 on a PC strip)", C_HARN, "Cable");
            nodrill(context, qCreatedBy(pid, EntityType.BODY));
        }

        // ================= rotating (built at turret angle 0) =================
        const rid = id + "rot";
        turned(context, rid, "brgI", "TT-12 Kaydon KC090XP0 bearing, inner race", P(ox, 0, 0), vector(0, 0, 1),
            [[BRGZ0, 4.5], [BRGZ0, 4.74], [BRGZ1, 4.74], [BRGZ1, 4.5]], 0, 0, C_ST, "Steel");
        nodrill(context, qCreatedBy(rid + "brgI", EntityType.BODY));
        {
            var holes = [];
            for (var i = 0; i < 8; i += 1)
                holes = append(holes, [ox + 4.3 * cos((45 * i) * degree), 4.3 * sin((45 * i) * degree), D10]);
            plate(context, rid, "clamp", "TT-13 Inner race clamp ring 1/8 6061", csXY(DECKT), concat(circPath(ox, 0, 4.74), []), 0.125,
                { "r" : 0, "holes" : concat([[ox, 0, 8.1]], holes) }, C_ALBK, "Aluminum 6061");
            for (var h in holes)
            {
                spacer(context, rid, "csp" ~ round(atan2(h[1], h[0] - ox) / degree + 180), "TT-14 Clamp spacer 0.30 OD", P(h[0], h[1], BRGZ0), P(h[0], h[1], BRGZ1), 0.15, 0.1, C_AL);
                br = append(br, [P(h[0], h[1], TPZ1), vector(0, 0, -1), 1.125, true]);      // heads on the plate, nuts under the clamp ring
            }
        }

        // TT-15 turret plate: 120T 10DP gear, 1/4 6061, ball passage, tapped holes for the plate angles and drum spokes
        const spokes = [70, 180, 290];
        {
            var holes = [[ox, 0, 6.5]];
            for (var i = 0; i < 8; i += 1)
                holes = append(holes, [ox + 4.3 * cos((45 * i) * degree), 4.3 * sin((45 * i) * degree), D10]);
            for (var sy in [1, -1])
                for (var x in [3.6, 4.9, -4.0, -5.0])
                    holes = append(holes, [ox + x, sy * 2.95, 0.159]);
            for (var sa in spokes)
                for (var t in [-0.22, 0.22])
                    holes = append(holes, [ox + 5.4 * cos(sa * degree) - t * sin(sa * degree), 5.4 * sin(sa * degree) + t * cos(sa * degree), 0.159]);
            plate(context, rid, "tp", "TT-15 Turret plate 120T 10DP 1/4 6061", csXY(BRGZ1), gearPts(ox, 0, 120, 10), 0.25,
                { "r" : 0, "holes" : holes }, C_ALRD, "Aluminum 6061");
        }

        // TT-60 cable drum + TT-61 spokes: the drum wall and lip carry the loop's inner run
        {
            const pid = rid + "drum";
            const hid = rid + "drumh";
            const c = new box(0);
            revolveSolid(context, nextId(pid, c), P(ox, 0, 0), vector(0, 0, 1),
                [[13.92, DRUMR0], [13.92, 7.5], [14.0, 7.5], [14.0, DRUMR1], [14.62, DRUMR1], [14.62, DRUMR0]]);
            const ab = WRAPBS * degree;
            sCyl(context, hid, c, P(ox + 6.9 * cos(ab), 6.9 * sin(ab), WRAPZ), P(ox + 7.4 * cos(ab), 7.4 * sin(ab), WRAPZ), 0.2);
            done(context, pid, hid, "TT-60 Cable drum (PETG print, 3 segments)", C_PETG, "PETG (40% infill)");
        }
        for (var sa in spokes)
        {
            const a = sa * degree;
            const u = vector(cos(a), sin(a), 0);
            const n = vector(-sin(a), cos(a), 0);
            const pid = rid + ("spk" ~ sa);
            const hid = rid + ("spkh" ~ sa);
            const c = new box(0);
            const cs = coordSystem(P(ox, 0, 0) + n * 0.4 * inch, u, -n);         // (u, v) = (radius, z), extruded across 0.8
            prismR(context, nextId(pid, c), cs, [[5.0, TPZ1], [5.85, TPZ1], [5.85, 13.92], [7.08, 13.92], [7.08, 14.17], [5.0, 14.17]], 0.04, 0, 0.8);     // bonded to the drum wall
            for (var t in [-0.22, 0.22])
            {
                const p = P(ox, 0, 0) + (u * 5.4 + n * t) * inch;
                sCyl(context, hid, c, p + vector(0, 0, 13.8) * inch, p + vector(0, 0, 14.3) * inch, D10 / 2);
                br = append(br, [p + vector(0, 0, 14.17) * inch, vector(0, 0, -1), 0.5]);
            }
            done(context, pid, hid, "TT-61 Drum spoke (PETG print)", C_PETG, "PETG (40% infill)");
        }

        // side plates (1/4 6061), local outline in (x, z); the lower front corner is cut back inside the drum
        const fx = ox + FLYX;
        const pvx = ox - 3.25;          // hood pivot
        const pvz = FLYZ;
        const hga = 216;                // pivot -> hood pinion, on a sector tooth gap
        const hmx = pvx + 1.525 * cos(hga * degree);      // 45T 20DP sector (r 1.125) + 16T pinion (r 0.4)
        const hmz = pvz + 1.525 * sin(hga * degree);
        const mx = ox - 1.6;            // flywheel motors
        const mz = 15.85;               // can bottoms (14.71) clear the service loop (top 14.605)
        const motso = [90, 150, 210, 270];                // X60 face holes used for the spacers (2.0 in circle)
        const hgso = [120, 180];        // hood gearbox standoffs about the gearbox axis (r 0.75), clear of the sector
        const so = [[ox + 5.7, 14.15, 0.1875], [ox + 6.2, 17.75, 0.25], [ox - 5.0, 17.5, 0.25]];   // plate standoffs [x, z, r] (the low front one clears the wheels)
        const outline = [[ox - 5.5, TPZ1], [ox + 6.0, TPZ1], [ox + 6.0, 14.7], [ox + 6.6, 15.3], [ox + 6.6, 18.3], [ox + 4.1, 21.4], [ox - 3.2, 21.4], [ox - 5.5, 19.0]];
        for (var sy in [1, -1])
        {
            var holes = [[fx, FLYZ, 1.125], [pvx, pvz, 0.257], [mx, mz, 0.6]];
            for (var a in motso)
                holes = append(holes, [mx + cos(a * degree), mz + sin(a * degree), D10]);
            for (var s in so)
                holes = append(holes, [s[0], s[1], D10]);
            for (var x in [ox + 3.6, ox + 4.9, ox - 4.0, ox - 5.0])
                holes = append(holes, [x, TPZ1 + 0.5, D10]);
            if (sy > 0)
            {
                for (var a in hgso)
                    holes = append(holes, [hmx + 0.75 * cos(a * degree), hmz + 0.75 * sin(a * degree), D10]);
                for (var q in [[ox + 1.4, 18.6], [ox + 3.6, 18.6]])
                    holes = append(holes, [q[0], q[1], D10]);
            }
            else
                holes = append(holes, [hmx, hmz, 0.6]);           // grommet for the hood X44 lead
            var bos = holeBosses(holes, 0.3);
            const pk = autoTruss(outline, bos, 1.55, 0.3, 0.32);
            plate(context, rid, "sp" ~ (sy > 0 ? "L" : "R"), "TT-1" ~ (sy > 0 ? 6 : 7) ~ " Shooter side plate " ~ (sy > 0 ? "L" : "R") ~ " 1/4 6061 (truss)",
                csXZ(sy > 0 ? 3.7 : -3.45), outline, 0.25, { "r" : 0.2, "holes" : holes, "pockets" : pk, "rp" : 0.1 }, C_ALRD, "Aluminum 6061");
            // plate-to-turret-plate angles (front and back, outside the ball path)
            for (var seg in [[ox + 3.3, ox + 5.3], [ox - 5.4, ox - 3.6]])
            {
                plate(context, rid, "ang" ~ (sy > 0 ? "L" : "R") ~ (seg[0] > ox ? "F" : "B"), "TT-18 Plate angle 1x1x1/8 6061", csYZ(seg[0]),
                    [[sy * 3.45, TPZ1], [sy * 2.45, TPZ1], [sy * 2.45, TPZ1 + 0.125], [sy * 3.325, TPZ1 + 0.125], [sy * 3.325, TPZ1 + 1.0], [sy * 3.45, TPZ1 + 1.0]],
                    seg[1] - seg[0], { "r" : 0 }, C_AL, "Aluminum 6061");
            }
            for (var x in [ox + 3.6, ox + 4.9, ox - 4.0, ox - 5.0])
            {
                br = append(br, [P(x, sy * 3.7, TPZ1 + 0.5), vector(0, -sy, 0), 0.625, true]);
                br = append(br, [P(x, sy * 2.95, TPZ1 + 0.125), vector(0, 0, -1), 0.375]);
            }
        }

        // standoffs between the side plates
        var si = 0;
        for (var s in so)
        {
            si += 1;
            spacer(context, rid, "pso" ~ si, "TT-19 Plate standoff " ~ (s[2] > 0.2 ? "1/2" : "3/8") ~ " OD x 6.9", P(s[0], -3.45, s[1]), P(s[0], 3.45, s[1]), s[2], 0.08, C_AL);
            br = append(br, [P(s[0], 3.7, s[1]), vector(0, -1, 0), 0.5]);
            br = append(br, [P(s[0], -3.7, s[1]), vector(0, 1, 0), 0.5]);
        }

        // back guide (1/8 6061): tangent continuation of the hood below its pivot, between the hood ribs
        plate(context, rid, "bg", "TT-20 Back guide 1/8 6061 (PTFE tape)", csYZ(ox - 3.125), [[-3.05, TPZ1 + 0.12], [3.05, TPZ1 + 0.12], [3.05, FLYZ - 0.1], [-3.05, FLYZ - 0.1]], 0.125,
            { "r" : 0.1 }, C_AL, "Aluminum 6061");

        // flywheel: 3x 4 in 60A wheels + 2 steel discs on 1/2 hex, bearings in the plates, pulleys outside
        hexShaft(context, rid, "fsh", "TT-21 Flywheel shaft 1/2 hex 9.1 in", P(fx, -4.55, FLYZ), P(fx, 4.55, FLYZ), 0.5, C_ST, "Steel");
        var wi = 0;
        for (var yc in [-1.85, 0.0, 1.85])
        {
            wi += 1;
            wheel(context, rid, "fw" ~ wi, "TT-22 Flywheel wheel 4 x 1.5 in 60A urethane", P(fx, yc, FLYZ), vector(0, 1, 0), 4.0, 1.5, 0.5, C_URE, "Urethane");
        }
        for (var yc in [-0.925, 0.925])
        {
            turned(context, rid, "fd" ~ (yc > 0 ? "L" : "R"), "TT-23 Flywheel mass disc 3.8 x 0.25 in 1018 steel", P(fx, yc, FLYZ), vector(0, 1, 0),
                [[-0.125, 0.2], [-0.125, 1.9], [0.125, 1.9], [0.125, 0.2]], 0.5, 0, C_ST, "Steel");
            nodrill(context, qCreatedBy(rid + ("fd" ~ (yc > 0 ? "L" : "R")), EntityType.BODY));
        }
        const rp18 = 18 * 5 / PI / 2 / 25.4;
        for (var sy in [1, -1])
        {
            const s = sy > 0 ? "L" : "R";
            turned(context, rid, "fsp" ~ s, "TT-24 Flywheel spacer 1/2 hex x 0.8", P(fx, sy * 2.6, FLYZ), vector(0, sy, 0),
                [[0, 0.2], [0, 0.42], [0.8, 0.42], [0.8, 0.2]], 0.5, 0, C_AL, "Aluminum 6061");
            nodrill(context, qCreatedBy(rid + ("fsp" ~ s), EntityType.BODY));
            fbearing(context, rid, "fb" ~ s, P(fx, sy * 3.604, FLYZ), vector(0, sy, 0), 1.125, 0.3125, 0.5, 0);
            htd(context, rid, "fp" ~ s, P(fx, sy * 4.02, FLYZ), vector(0, 1, 0), 18, 0.354, 0.5);
            collar(context, rid, "fc" ~ s, P(fx, sy * 4.4, FLYZ), vector(0, 1, 0), 1.125, 0.2, 0.5, 0);
            // X60 1:1 on four spacers (on its own face holes, clear of the 18T pulley and belt), bolted from inside the plate
            kraken(context, rid + ("fm" ~ s), P(mx, sy * 4.45, mz), vector(0, -sy, 0), true);
            htd(context, rid, "mp" ~ s, P(mx, sy * 4.02, mz), vector(0, 1, 0), 18, 0.354, 0.315);
            beltLoop(context, rid, "fbelt" ~ s, "TT-25 Flywheel belt 9 mm HTD 5 mm 1:1 (18T:18T)", P(mx, sy * 4.02, mz), rp18, P(fx, sy * 4.02, FLYZ), rp18, vector(0, 1, 0), 0.354);
            for (var a in motso)
            {
                const u = mx + cos(a * degree);
                const w = mz + sin(a * degree);
                spacer(context, rid, "fms" ~ s ~ a, "TT-26 Motor spacer 0.375 OD x 0.75", P(u, sy * 3.7, w), P(u, sy * 4.45, w), 0.1875, 0.1, C_AL);
                br = append(br, [P(u, sy * 3.45, w), vector(0, sy, 0), 1.25]);
            }
        }

        // ---- hood (sub-group, pivots about the back point of the flywheel arc)
        const hid = rid + "hood";
        {
            const r0 = HOODR;
            const r1 = HOODR + 0.125;
            var pts = [[fx - r0, FLYZ]];
            pts = concat(pts, arcTo(fx, FLYZ, r0, 180, 135));
            pts = append(pts, [fx + r1 * cos(135 * degree), FLYZ + r1 * sin(135 * degree)]);
            pts = concat(pts, arcTo(fx, FLYZ, r1, 135, 180));
            plate(context, hid, "hood", "TT-30 Hood 1/8 polycarb + PTFE liner", csXZ(3.15), pts, 6.3, { "r" : 0 }, C_PCX, "Polycarbonate");
        }
        for (var sy in [1, -1])
        {
            // rib: band behind the hood + pivot lobe (the left rib carries a 45T 20DP sector, teeth at 196..236 deg)
            const ra = HOODR + 0.125;
            const rb = HOODR + 0.55;
            var band = [[fx + ra * cos(182 * degree), FLYZ + ra * sin(182 * degree)]];
            band = concat(band, arcTo(fx, FLYZ, ra, 182, 138));
            band = append(band, [fx + rb * cos(138 * degree), FLYZ + rb * sin(138 * degree)]);
            band = concat(band, arcTo(fx, FLYZ, rb, 138, 182));
            var lobe = [];
            if (sy > 0)
            {
                lobe = arcPts(pvx, pvz, 0.5, -60, 188, 16);
                const rp = 1.125;
                const pa = 8.0;
                for (var a = 196; a <= 236 + 1e-6; a += pa)
                {
                    lobe = append(lobe, [pvx + (rp - 0.0625) * cos((a - 0.30 * pa) * degree), pvz + (rp - 0.0625) * sin((a - 0.30 * pa) * degree)]);
                    lobe = append(lobe, [pvx + (rp + 0.05) * cos((a - 0.12 * pa) * degree), pvz + (rp + 0.05) * sin((a - 0.12 * pa) * degree)]);
                    lobe = append(lobe, [pvx + (rp + 0.05) * cos((a + 0.12 * pa) * degree), pvz + (rp + 0.05) * sin((a + 0.12 * pa) * degree)]);
                    lobe = append(lobe, [pvx + (rp - 0.0625) * cos((a + 0.30 * pa) * degree), pvz + (rp - 0.0625) * sin((a + 0.30 * pa) * degree)]);
                }
                lobe = concat(lobe, arcPts(pvx, pvz, 0.5, 246, 300, 6));
            }
            else
                lobe = arcPts(pvx, pvz, 0.5, 0, 345, 24);
            const cs = csXZ(sy > 0 ? 3.325 : -3.2);
            const pid = hid + ("rib" ~ (sy > 0 ? "L" : "R"));
            const hh = hid + ("ribh" ~ (sy > 0 ? "L" : "R"));
            const c = new box(0);
            prismR(context, nextId(pid, c), cs, band, 0, 0, 0.125);
            prismR(context, nextId(pid, c), cs, lobe, 0, 0, 0.125);
            sCyl(context, hh, c, P(pvx, -4.0, pvz), P(pvx, 4.0, pvz), 0.128);
            for (var a in [150, 170])
                sCyl(context, hh, c, P(fx + 7.65 * cos(a * degree), -4.0, FLYZ + 7.65 * sin(a * degree)), P(fx + 7.65 * cos(a * degree), 4.0, FLYZ + 7.65 * sin(a * degree)), D10 / 2);
            done(context, pid, hh, "TT-31 Hood rib " ~ (sy > 0 ? "L, 45T 20DP sector" : "R") ~ " 1/8 6061", C_AL, "Aluminum 6061");
            br = append(br, [P(pvx, sy * 3.7, pvz), vector(0, -sy, 0), 0.5]);
        }
        spinAxis(context, rid + "hoodRot", qCreatedBy(hid, EntityType.BODY), P(pvx, 0, pvz), vector(0, 1, 0), -definition.hood);

        // hood drive: X44 + 25:1 MAXPlanetary across the back of the turret, 16T 20DP pinion on the
        // gearbox output (no outboard bearing: the MAXPlanetary output carries it), gearbox on two
        // standoffs from the left plate
        kraken(context, rid + "hmot", P(hmx, -0.3, hmz), vector(0, 1, 0), false);
        turned(context, rid, "hgb", "TT-33 REV MAXPlanetary 5:1 + 5:1", P(hmx, -0.3, hmz), vector(0, 1, 0),
            [[0, 0.4], [0, 0.88], [2.55, 0.88], [2.55, 0.95], [2.7, 0.95], [2.7, 0.3], [0.25, 0.3], [0.25, 0.4]], 0, 0, C_BLK, "Aluminum 6061");
        hexShaft(context, rid, "hsh", "TT-34 Hood output shaft 1/2 hex", P(hmx, 2.4, hmz), P(hmx, 3.4, hmz), 0.5, C_ST, "Steel");
        {
            const pid = rid + "hpin";
            const hid2 = rid + "hpinh";
            const c = new box(0);
            const cs = coordSystem(P(hmx, 3.325, hmz), vector(1, 0, 0), vector(0, -1, 0));     // (u, v) = (x, z), extruded -Y
            // a pinion tooth points at the pivot (into the sector gap at hga); the sector turns +hood, the pinion -45/16 x hood
            const hr = (hga - 180 - 22.5) - definition.hood / degree * 45 / 16;
            sPoly(context, pid, c, cs, gearPtsR(0, 0, 16, 20, hr), 0, 0.125);
            sPoly(context, hid2, c, cs, hexPts(0, 0, 0.5), -0.1, 0.3);
            done(context, pid, hid2, "TT-35 Hood pinion 16T 20DP steel", C_ST, "Steel");
            nodrill(context, qCreatedBy(pid, EntityType.BODY));
        }
        for (var a in hgso)
        {
            const q = [hmx + 0.75 * cos(a * degree), hmz + 0.75 * sin(a * degree)];
            spacer(context, rid, "hso" ~ a, "TT-36 Gearbox standoff 0.375 OD x 1.05", P(q[0], 2.4, q[1]), P(q[0], 3.45, q[1]), 0.1875, 0.1, C_AL);
            br = append(br, [P(q[0], 3.7, q[1]), vector(0, -1, 0), 1.5]);
        }

        // turret Limelight 4 on the left plate (holes at ox + 1.4 / ox + 3.6, z 18.6)
        br = concat(br, ll4Turret(context, rid, ox));

        // ---- turret wiring: from the loop's moving end in through the drum, under the left X60 and
        //      around the back of the plates to the right side; tails to both X60s, the hood X44
        //      (through a grommet in the right plate) and the Limelight
        {
            const b = WRAPBS * degree;
            const p0 = [ox + WRAPRI * cos(b), WRAPRI * sin(b), WRAPZ];
            const p1 = [ox + 6.4 * cos(b), 6.4 * sin(b), WRAPZ];
            cable(context, rid, "wr1", "TT-50 Turret harness (loop end -> under the left X60 -> around the back -> right side)",
                [p0, p1, [ox - 3.0, 5.6, 14.45], [ox - 5.7, 3.9, 14.45], [ox - 5.7, -3.9, 14.45], [ox - 3.0, -5.6, 14.45]], 0.3, C_HARN);
            cable(context, rid, "wr2", "TT-51 Flywheel X60 L tail", [[ox - 3.0, 5.6, 14.45], [ox - 3.0, 5.6, 15.35], [ox - 2.95, 6.9, mz]], 0.24, C_HARN);
            cable(context, rid, "wr3", "TT-52 Flywheel X60 R tail", [[ox - 3.0, -5.6, 14.45], [ox - 3.0, -5.6, 15.35], [ox - 2.95, -6.9, mz]], 0.24, C_HARN);
            cable(context, rid, "wr4", "TT-53 Hood X44 tail (through the right plate grommet)", [[hmx, -3.3, hmz], [hmx, -4.05, hmz], [ox - 5.7, -4.05, 14.6]], 0.2, C_HARN);
            cable(context, rid, "wr5", "TT-54 Turret LL4 Ethernet (PoE)", [p1, [ox, 5.8, WRAPZ], [ox, 5.8, 17.3], [ox + 0.9, 5.8, 17.55]], 0.22, C_ETH);
        }

        bolts(context, rid, "hw", br);
        spinZ(context, id + "spin", qCreatedBy(rid, EntityType.BODY), ox, 0, definition.turret);
    });
