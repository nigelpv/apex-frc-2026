
// =========================================================================
// 04 HOPPER + UPTAKE: 1x1 uprights on the side / rear rails with base
// gussets, 1x1 top rails (the hopper sides are the intake towers, which run
// back to the mid uprights), 1/4 truss-pocketed turret deck (front edge
// follows the turret fence), 1x1 deck beam, 1/16 polycarb fence around the
// turret cable-wrap ring, rolled 1/16 polycarb funnel (15 deg) into the bowl.
// Uptake: single-sided twin 15 mm belts (30T HTD) on the back side of the
// rear pocket against a fixed UHMW wall; X44 direct on the upper shaft.
// =========================================================================
annotation { "Feature Type Name" : "APEX 04 Hopper + Uptake" }
export const apexHopper = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        var bl = [];
        // ---- uprights, top rails, base gussets
        var ui = 0;
        for (var sy in [1, -1])
        {
            for (var x0 in [3.75, -0.75])
            {
                ui += 1;
                tube(context, id, "up" ~ ui, (x0 > 0 ? "HP-01 Front upright" : "HP-02 Mid upright") ~ " 1x1x1/16 (16.25 in)", P(x0 - 0.5, sy > 0 ? 13.0 : -14.0, RZ1), P(x0 + 0.5, sy > 0 ? 14.0 : -13.0, 20.0), TW, true, C_ALBK);
                // gusset bolts land on the 0.25 / 0.75 hole grid of the rail and the upright
                const gb = [[x0 - 1.0, 2.75], [x0, 2.75], [x0 + 0.5, 2.75], [x0, 4.625], [x0, 5.625]];
                var gh = [];
                for (var q in gb)
                    gh = append(gh, [q[0], q[1], D10]);
                plate(context, id, "gus" ~ ui, "HP-1" ~ (x0 > 0 ? 1 : 2) ~ " Upright base gusset 1/8 6061", csXZ(sy * 13.0 + (sy > 0 ? 0 : 0.125)),
                    [[x0 - 1.5, RZ0 + 0.15], [x0 + 0.85, RZ0 + 0.15], [x0 + 0.85, RZ1 + 0.6], [x0 + 0.55, 6.6], [x0 - 0.55, 6.6], [x0 - 1.5, RZ1 + 0.6]], 0.125,
                    { "r" : 0.15, "holes" : gh }, C_AL, "Aluminum 6061");
                for (var q in gb)
                    bl = append(bl, [P(q[0], sy * 12.875, q[1]), vector(0, sy, 0), 0.25]);
            }
            tube(context, id, "tr" ~ (sy > 0 ? "L" : "R"), "HP-07 Top rail 1x1x1/16 (13.55 in)", P(-1.25, sy > 0 ? 13.0 : -14.0, 20.0), P(12.3, sy > 0 ? 14.0 : -13.0, 21.0), TW, true, C_ALBK);
            // rear uprights (deck supports) on the rear rail between the rear modules
            ui += 1;
            tube(context, id, "up" ~ ui, "HP-05 Rear deck post 1x1x1/16 (9.0 in)", P(-13.5, sy * 6.6 - 0.5, RZ1), P(-12.5, sy * 6.6 + 0.5, DECKZ), TW, true, C_ALBK);
            {
                const yc = sy * 6.6;
                const gb = [[sy * 5.75, 2.75], [sy * 6.75, 2.75], [sy * 6.6, 4.5], [sy * 6.6, 5.5]];
                var gh = [];
                for (var q in gb)
                    gh = append(gh, [q[0], q[1], D10]);
                plate(context, id, "gus" ~ ui, "HP-16 Rear post gusset 1/8 6061", csYZ(-12.5),
                    [[yc - sy * 1.3, RZ0 + 0.15], [yc + sy * 0.95, RZ0 + 0.15], [yc + sy * 0.95, RZ1 + 0.6], [yc + sy * 0.55, 6.6], [yc - sy * 0.55, 6.6], [yc - sy * 1.3, RZ1 + 0.6]], 0.125,
                    { "r" : 0.15, "holes" : gh }, C_AL, "Aluminum 6061");
            }
            for (var q in [[sy * 5.75, 2.75], [sy * 6.75, 2.75], [sy * 6.6, 4.5], [sy * 6.6, 5.5]])
                bl = append(bl, [P(-12.375, q[0], q[1]), vector(-1, 0, 0), 0.25]);

            // (hopper sides: the intake towers run back to the mid uprights as 1/8 6061 liners; a polycarb
            //  skin outside the uprights would stand 1/16 outside the FRAME PERIMETER)
            // deck support angle on the mid upright
            plate(context, id, "dsa" ~ (sy > 0 ? "L" : "R"), "HP-17 Beam support angle 1x1x1/8 x 1.0", csYZ(-1.25),
                [[sy * 13.0, DECKZ - 2.0], [sy * 12.875, DECKZ - 2.0], [sy * 12.875, DECKZ - 1.125], [sy * 12.0, DECKZ - 1.125], [sy * 12.0, DECKZ - 1.0], [sy * 13.0, DECKZ - 1.0]], 1.0,
                { "r" : 0 }, C_AL, "Aluminum 6061");
            bl = append(bl, [P(-0.75, sy * 12.5, DECKZ - 1.125), vector(0, 0, 1), 0.25]);
            bl = append(bl, [P(-0.75, sy * 12.875, DECKZ - 1.5), vector(0, sy, 0), 0.25]);
        }

        // ---- deck beam under the deck front (also carries the uptake wall)
        {
            var bb = [];
            for (var y in BEAMBOLTS)
                bb = append(bb, [P(-1.0, y, DECKZ - 0.5), vector(0, 0, 1), D10]);
            tube2(context, id, "beam", "HP-18 Deck beam 1x1x1/16 (26.0 in)", P(-1.25, -13.0, DECKZ - 1.0), P(-0.25, 13.0, DECKZ), TW, true, bb, C_ALBK);
        }

        // ---- turret deck: 1/4 6061, front edge follows the fence, bore for the bearing housing,
        //      truss-pocketed around every fixing
        {
            const th = acos(-TURX / DECKR);
            var outline = [[-13.4, -12.95], [0.0, -12.95], [0.0, -DECKR * sin(th)]];
            outline = concat(outline, arcTo(TURX, 0, DECKR, -th / degree, th / degree));
            outline = concat(outline, [[0.0, DECKR * sin(th)], [0.0, 12.95], [-13.4, 12.95]]);
            var poly = [[-13.4, -12.95], [0.0, -12.95]];                 // sampled outline for the truss layout
            poly = concat(poly, arcPts(TURX, 0, DECKR, -th / degree, th / degree, 24));
            poly = concat(poly, [[0.0, 12.95], [-13.4, 12.95]]);
            var holes = [[TURX, 0, 9.7]];
            for (var i = 0; i < 8; i += 1)
                holes = append(holes, [TURX + 5.35 * cos((22.5 + 45 * i) * degree), 5.35 * sin((22.5 + 45 * i) * degree), D10]);
            for (var y in BEAMBOLTS)
                holes = append(holes, [-1.0, y, D10]);
            // turret encoder pod bolts (19T, 21T)
            for (var q in [[ENC19A, 19], [ENC21A, 21]])
            {
                const rr = 6.0 + q[1] / 20 + 0.01;
                for (var sx in [-1.08, 1.08])
                    holes = append(holes, [TURX + rr * cos(q[0] * degree) + sx, rr * sin(q[0] * degree), D10]);
            }
            for (var y in [-6.2, 6.2])
                holes = append(holes, [TURX - UPP, y, D10]);
            for (var y in [-6.6, 6.6])
                holes = append(holes, [-13.0, y, D10]);
            // yaw drive: gearbox output / pinion clearance hole
            holes = append(holes, [TURX + 6.61 * cos(YAWA * degree), 6.61 * sin(YAWA * degree), 1.4]);
            for (var p in ringPosts())
                holes = append(holes, [p[0], p[1], D10]);                // cable-wrap ring posts
            holes = concat(holes, elecDeckHoles());
            var bos = holeBosses(holes, 0.3);
            bos = append(bos, [TURX, 0, 6.05]);                          // bearing housing seat
            bos = append(bos, [-11.9, 0, 3.0]);                          // breaker pocket (cut below)
            var pk = autoTruss(poly, bos, 2.3, 0.35, 0.4);
            pk = append(pk, rrect(-13.05, -2.6, -10.75, 2.6, 0.3));    // breaker + battery leads
            plate(context, id, "deck", "HP-20 Turret deck 1/4 6061 (truss pocketed)", csXY(DECKZ), outline, 0.25,
                { "r" : 0.2, "holes" : holes, "pockets" : pk, "rp" : 0.2 }, C_AL, "Aluminum 6061");
            for (var y in BEAMBOLTS)
                bl = append(bl, [P(-1.0, y, DECKT), vector(0, 0, -1), 1.5, true]);
            for (var y in [-6.6, 6.6])
                bl = append(bl, [P(-13.0, y, DECKT), vector(0, 0, -1), 0.5]);
        }

        // ---- fence: 1/16 polycarb arc around the turret swing, wings back to the side walls
        {
            const ti = acos(3.75 / FENR);
            const to = acos(3.8125 / (FENR + 0.0625));
            var pts = [[-0.6875, -12.95], [-0.6875, -(FENR + 0.0625) * sin(to)]];
            pts = concat(pts, arcTo(TURX, 0, FENR + 0.0625, -to / degree, to / degree));
            pts = concat(pts, [[-0.6875, 12.95], [-0.75, 12.95], [-0.75, FENR * sin(ti)]]);
            pts = concat(pts, arcTo(TURX, 0, FENR, ti / degree, -ti / degree));
            pts = append(pts, [-0.75, -12.95]);
            plate(context, id, "fence", "HP-22 Turret fence 1/16 polycarb (bent: R8.96 arc + wings)", csXY(DECKT), pts, 8.0, { "r" : 0 }, C_PCX, "Polycarbonate");
        }

        // ---- funnel: 15 deg cone about the rotor axis, trimmed to the hopper
        {
            const pid = id + "fun";
            const c = new box(0);
            const t = tan(15 * degree);
            revolveSolid(context, nextId(pid, c), P(ROTX, 0, 0), vector(0, 0, 1),
                [[BOWLZ1, BOWLR + 0.06], [BOWLZ1 + (18.5 - BOWLR) * t, 18.5], [BOWLZ1 + (18.5 - BOWLR) * t + 0.065, 18.5], [BOWLZ1 + 0.065, BOWLR + 0.06]]);
            fCuboid(context, id + "funbox", { "corner1" : P(-4.5, -11.6, BOWLZ1 - 0.2), "corner2" : P(12.3, 11.6, 12.6) });
            opBoolean(context, id + "funcut", { "tools" : qUnion([qCreatedBy(pid, EntityType.BODY), qCreatedBy(id + "funbox", EntityType.BODY)]), "operationType" : BooleanOperationType.INTERSECTION });
            const q = qUnion([qCreatedBy(pid, EntityType.BODY), qCreatedBy(id + "funbox", EntityType.BODY), qCreatedBy(id + "funcut", EntityType.BODY)]);
            setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : "HP-25 Funnel 1/16 polycarb (rolled cone, 15 deg)" });
            setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : C_PCX });
            setProperty(context, { "entities" : q, "propertyType" : PropertyType.MATERIAL, "value" : mat("Polycarbonate") });
        }

        // ---- UPTAKE (fixed, hangs under the deck)
        const ux = TURX - UPP;                  // back pulley axis X
        for (var sy in [1, -1])
        {
            tube2(context, id, "ut" ~ (sy > 0 ? "L" : "R"), "UP-0" ~ (sy > 0 ? 1 : 2) ~ " Uptake column 1x1x1/16 " ~ (sy > 0 ? "L" : "R"),
                P(ux - 0.5, sy > 0 ? UPY - 0.5 : -UPY - 0.5, 6.9), P(ux + 0.5, sy > 0 ? UPY + 0.5 : -UPY + 0.5, DECKZ - 0.4), TW, true,
                [[P(ux, sy * UPY, UPZ0), vector(0, 1, 0), 0.875], [P(ux, sy * UPY, UPZ1), vector(0, 1, 0), 0.875]], C_ALBK);
            for (var z in [UPZ0, UPZ1])
                fbearing(context, id, "ub" ~ (sy > 0 ? "L" : "R") ~ (z > 9 ? "T" : "B"), P(ux, sy * 1.326, z), vector(0, -sy, 0), 0.875, 0.3125, 0.375, 0);
            for (var z in [UPZ0, UPZ1])
            {
                const k = (sy > 0 ? "L" : "R") ~ (z > 9 ? "T" : "B");
                htd(context, id, "upl" ~ k, P(ux, sy * 0.6, z), vector(0, 1, 0), 30, 0.59, 0.375);
            }
            beltLoop(context, id, "ubelt" ~ (sy > 0 ? "L" : "R"), "UP-07 Uptake belt 15 mm HTD 5 mm (rubber-backed) " ~ (sy > 0 ? "L" : "R"),
                P(ux, sy * 0.6, UPZ0), 0.94, P(ux, sy * 0.6, UPZ1), 0.94, vector(0, 1, 0), 0.59);
            bl = append(bl, [P(ux, sy * UPY, DECKZ - 0.15), vector(0, 0, -1), 0.5]);
        }
        plate(context, id, "ubar", "UP-03 Uptake hanger bar 1/4 6061", csXY(DECKZ - 0.4), [[ux - 0.5, -6.6], [ux + 0.5, -6.6], [ux + 0.5, 6.6], [ux - 0.5, 6.6]], 0.25,
            { "r" : 0.2, "holes" : [[ux, -6.2, D10], [ux, 6.2, D10], [ux, -UPY, D10], [ux, UPY, D10]] }, C_AL, "Aluminum 6061");
        for (var y in [-6.2, 6.2])
        {
            spacer(context, id, "usp" ~ (y > 0 ? "L" : "R"), "UP-10 Spacer 0.375 OD x 0.15", P(ux, y, DECKZ - 0.15), P(ux, y, DECKZ), 0.1875, 0.1, C_AL);
            bl = append(bl, [P(ux, y, DECKT), vector(0, 0, -1), 0.9, true]);
        }
        hexShaft(context, id, "ushB", "UP-04 Uptake lower shaft 3/8 hex", P(ux, -2.35, UPZ0), P(ux, 2.35, UPZ0), 0.375, C_ST, "Steel");
        hexShaft(context, id, "ushT", "UP-05 Uptake upper shaft 3/8 hex (driven)", P(ux, -2.35, UPZ1), P(ux, 2.3, UPZ1), 0.375, C_ST, "Steel");
        // X44 direct drive on the upper shaft (+Y side, behind the bowl)
        kraken(context, id + "umot", P(ux, 3.8, UPZ1), vector(0, -1, 0), false);
        turned(context, id, "ucpl", "UP-06 Shaft coupler 8 mm keyed -> 3/8 hex", P(ux, 2.25, UPZ1), vector(0, 1, 0), [[0, 0.25], [0, 0.35], [1.35, 0.35], [1.35, 0.25]], 0, 0, C_ST, "Steel");
        // UP-08 bent bracket: motor face plate + top flange bolted up into the hanger bar, one spacer to
        // the column below the motor can (the X44 can is 1.73 in across, so nothing else fits beside it)
        {
            const pid = id + "umnt";
            const hid = id + "umnth";
            const c = new box(0);
            sBox(context, pid, c, P(ux - 0.9, 3.675, 9.45), P(ux + 0.9, 3.8, DECKZ - 0.4));              // face plate
            sBox(context, pid, c, P(ux - 0.9, 3.8, DECKZ - 0.525), P(ux + 0.9, 4.95, DECKZ - 0.4));       // top flange
            sCyl(context, hid, c, P(ux, 3.6, UPZ1), P(ux, 3.9, UPZ1), 0.45);                              // pilot bore
            for (var a in [30, 150, 210, 330])                                                           // X44 face: 1.375 circle
                sCyl(context, hid, c, P(ux + 0.6875 * cos(a * degree), 3.6, UPZ1 + 0.6875 * sin(a * degree)), P(ux + 0.6875 * cos(a * degree), 3.9, UPZ1 + 0.6875 * sin(a * degree)), D10 / 2);
            sCyl(context, hid, c, P(ux, 3.6, 9.875), P(ux, 3.9, 9.875), D10 / 2);
            for (var y in [4.2, 4.7])
                sCyl(context, hid, c, P(ux, y, DECKZ - 0.6), P(ux, y, DECKZ - 0.3), D10 / 2);
            done(context, pid, hid, "UP-08 Uptake motor bracket 1/8 6061 (bent)", C_ALBK, "Aluminum 6061");
        }
        for (var a in [30, 150, 210, 330])
            bl = append(bl, [P(ux + 0.6875 * cos(a * degree), 3.675, UPZ1 + 0.6875 * sin(a * degree)), vector(0, 1, 0), 0.375]);     // into the X44 face
        spacer(context, id, "ums", "UP-09 Motor bracket spacer 0.375 OD x 1.45", P(ux, UPY + 0.5, 9.875), P(ux, 3.675, 9.875), 0.1875, 0.1, C_AL);
        bl = append(bl, [P(ux, 3.8, 9.875), vector(0, -1, 0), 1.625]);                 // into a rivnut in the column wall
        for (var y in [4.2, 4.7])
            bl = append(bl, [P(ux, y, DECKZ - 0.525), vector(0, 0, 1), 0.375]);        // up into the tapped hanger bar
        // fixed UHMW wall on the front side of the column, hung from the deck beam
        plate(context, id, "uwall", "UP-11 Uptake wall 1/4 UHMW", csYZ(TURX + UPB), [[-2.3, FINZ + 0.15], [2.3, FINZ + 0.15], [2.3, 11.6], [-2.3, 11.6]], 0.25,
            { "r" : 0.4, "holes" : [[-1.6, 11.2, D10], [1.6, 11.2, D10], [-1.6, 10.6, D10], [1.6, 10.6, D10]] }, C_UHMW, "UHMW");
        for (var sy in [1, -1])
        {
            plate(context, id, "uwb" ~ (sy > 0 ? "L" : "R"), "UP-12 Uptake wall hanger 1/8 6061", csXZ(sy * 1.6 + 0.375),
                [[TURX + UPB + 0.25, 10.2], [TURX + UPB + 0.375, 10.2], [TURX + UPB + 0.375, DECKZ - 1.125], [-0.25, DECKZ - 1.125], [-0.25, DECKZ - 1.0], [TURX + UPB + 0.25, DECKZ - 1.0]], 0.75,
                { "r" : 0 }, C_AL, "Aluminum 6061");
            for (var z in [10.6, 11.2])
                bl = append(bl, [P(TURX + UPB + 0.375, sy * 1.6, z), vector(-1, 0, 0), 0.35]);
            bl = append(bl, [P(-0.75, sy * 1.75, DECKZ - 1.125), vector(0, 0, 1), 0.25]);
        }
        bolts(context, id, "hw", bl);
    });
