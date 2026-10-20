
// =========================================================================
// 06 INTAKE: pivoting over-bumper intake, 23 in throat. Arm pivots about a
// 1/2 hex shaft at (12.75, 10.21) carried in 1/8 6061 towers that double as
// the hopper's front side liners. Deployed 37 deg: 2 in silicone front roller
// 7.6 in past the frame, FUEL rides a 1/8 polycarb bed under two powered 2 in
// top rollers and drops into the hopper over the pivot. Stowed (+127 deg) the
// bed stands vertical and closes the hopper front (top at 21.71 in).
// Rollers: X44 -> 12T:28T -> rear top roller -> 1:1 -> front top -> 1:1 ->
// front roller (left belt band). Deploy: X44 + 25:1 MAXPlanetary low in front
// of the bowl, 12T:36T belt outside the right tower = 75:1.
// Arm frame: s along the arm (pivot -> front roller), n normal (up when deployed).
// =========================================================================
function armP(s is number, n is number, y is number) returns Vector
{
    return P(IOX + s * cos(ITH) + n * sin(ITH), y, IOZ - s * sin(ITH) + n * cos(ITH));
}

// sketch frame in the arm plane: u = s, v = n, extrude toward -Y from y0
function armCS(y0 is number) returns CoordSystem
{
    return coordSystem(P(IOX, y0, IOZ), vector(cos(ITH), 0, -sin(ITH)), vector(0, -1, 0));
}

annotation { "Feature Type Name" : "APEX 06 Intake" }
export const apexIntake = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Deployed", "Default" : true }
        definition.deployed is boolean;
    }
    {
        const aid = id + "arm";
        var ba = [];        // hardware that moves with the arm
        var bf = [];        // fixed hardware
        const T1 = [7.0, 6.5];      // front top roller
        const T2 = [2.6, 6.5];      // rear top roller
        const RM = [5.2, 8.4];      // roller motor
        const TIE = [3.3, 8.6];
        const XT = [[1.2, -0.375], [8.6, -0.375]];      // bed cross tubes
        const yA = 11.78;           // belt layer A (motor -> T2, T1 -> front roller)
        const yB = 12.29;           // belt layer B (T2 -> T1)

        // ---- arm plates (1/4 7075, truss)
        const outline = [[-0.95, -0.3], [9.2, -0.3], [9.75, -1.0], [10.9, -1.0], [11.3, -0.5], [11.3, 1.1], [8.7, 7.9], [6.8, 9.3], [3.0, 9.3], [1.9, 7.6], [1.5, 1.6], [-0.95, 1.0]];
        for (var sy in [1, -1])
        {
            var holes = [[IL, 0, 1.125], [T1[0], T1[1], 1.125], [T2[0], T2[1], 1.125], [TIE[0], TIE[1], D10]];
            for (var x in XT)
                holes = append(holes, [x[0], x[1], D10]);
            if (sy > 0)
            {
                holes = append(holes, [RM[0], RM[1], 0.7]);
                for (var i = 0; i < 4; i += 1)
                    holes = append(holes, [RM[0] + 0.9 * cos((45 + 90 * i) * degree), RM[1] + 0.9 * sin((45 + 90 * i) * degree), D10]);
            }
            var bos = holeBosses(holes, 0.3);
            bos = append(bos, [0, 0, 0.8]);
            const pk = autoTruss(outline, bos, 1.7, 0.32, 0.34);
            plate(context, aid, "pl" ~ (sy > 0 ? "L" : "R"), "IN-0" ~ (sy > 0 ? 1 : 2) ~ " Intake arm plate " ~ (sy > 0 ? "L" : "R") ~ " 1/4 7075 (truss)",
                armCS(sy > 0 ? IYP1 : -IYP0), outline, 0.25, { "r" : 0.2, "holes" : holes, "hex" : [[0, 0, 0.5]], "pockets" : pk, "rp" : 0.1 }, C_BLUE, "Aluminum 7075");
            // roller bearings, flange outboard
            for (var q in [[IL, 0], T1, T2])
                fbearing(context, aid, "b" ~ (sy > 0 ? "L" : "R") ~ round(q[0] * 10), armP(q[0], q[1], sy * 12.704), vector(0, sy, 0), 1.125, 0.3125, 0.5, 0);
        }

        // ---- shafts and rollers
        hexShaft(context, aid, "piv", "IN-03 Arm pivot shaft 1/2 hex 7075", armP(0, 0, -13.75), armP(0, 0, 13.15), 0.5, C_AL, "Aluminum 7075");
        var ri = 0;
        for (var q in [[IL, 0, "IN-04 Front roller"], [T1[0], T1[1], "IN-05 Top roller (front)"], [T2[0], T2[1], "IN-06 Top roller (rear)"]])
        {
            ri += 1;
            hexShaft(context, aid, "sh" ~ ri, q[2] ~ " shaft 1/2 hex", armP(q[0], q[1], -12.86), armP(q[0], q[1], 12.86), 0.5, C_ST, "Steel");
            turned(context, aid, "rl" ~ ri, q[2] ~ " 2.0 in (1.875 polycarb tube + 1/16 silicone)", armP(q[0], q[1], 0), vector(0, 1, 0),
                [[-11.45, 0.86], [-11.45, 0.98], [-11.4, 1.0], [11.4, 1.0], [11.45, 0.98], [11.45, 0.86]], 0, 0, C_SIL, "Silicone");
            for (var sy in [1, -1])
                turned(context, aid, "hub" ~ ri ~ (sy > 0 ? "L" : "R"), "IN-07 Roller end hub (PETG print)", armP(q[0], q[1], sy * 11.45), vector(0, -sy, 0),
                    [[-0.08, 0.2], [-0.08, 1.0], [0, 1.0], [0, 0.86], [0.6, 0.86], [0.6, 0.2]], 0.5, 0, C_PETG, "PETG (40% infill)");
        }

        // ---- bed + cross tubes + tie
        plate(context, aid, "bed", "IN-08 Intake bed 1/8 polycarb (PTFE tape)", coordSystem(armP(0, -0.125, 0), vector(cos(ITH), 0, -sin(ITH)), vector(sin(ITH), 0, cos(ITH))),
            [[0.4, -IYW], [9.4, -IYW], [9.4, IYW], [0.4, IYW]], 0.125, { "r" : 0.3, "holes" : [[1.2, -6.0, D10], [1.2, 6.0, D10], [8.6, -6.0, D10], [8.6, 6.0, D10]] }, C_PCX, "Polycarbonate");
        var xi = 0;
        for (var x in XT)
        {
            xi += 1;
            spacer(context, aid, "xt" ~ xi, "IN-09 Bed cross tube 1/2 OD 6061", armP(x[0], x[1], -IYP0), armP(x[0], x[1], IYP0), 0.25, 0.08, C_AL);
            for (var sy in [1, -1])
            {
                ba = append(ba, [armP(x[0], x[1], sy * (IYP1)), vector(0, -sy, 0), 0.5, false, true]);              // flush: the tower is 0.075 away
                ba = append(ba, [armP(x[0], 0, sy * 6.0), -vector(sin(ITH), 0, cos(ITH)), 0.375, false, true]);       // flush under the FUEL
            }
        }
        spacer(context, aid, "tie", "IN-10 Arm tie tube 1/2 OD 6061", armP(TIE[0], TIE[1], -IYP0), armP(TIE[0], TIE[1], IYP0), 0.25, 0.08, C_AL);
        for (var sy in [1, -1])
            ba = append(ba, [armP(TIE[0], TIE[1], sy * IYP1), vector(0, -sy, 0), 0.5, false, true]);

        // ---- roller drive (left band): X44 -> 12T:28T -> T2 -> 24T:24T -> T1 -> 24T:24T -> front roller
        kraken(context, aid + "rmot", armP(RM[0], RM[1], 11.3), vector(0, 1, 0), false);
        // X44 face holes are on a 1.375 circle every 30 deg of world XZ angle; in the arm frame that is 7 + 30k deg
        var rmh = [[RM[0], RM[1], 0.9]];
        for (var i = 0; i < 4; i += 1)
            rmh = append(rmh, [RM[0] + 0.9 * cos((45 + 90 * i) * degree), RM[1] + 0.9 * sin((45 + 90 * i) * degree), D10]);
        for (var a in [7, 97, 187, 277])
        {
            rmh = append(rmh, [RM[0] + 0.6875 * cos(a * degree), RM[1] + 0.6875 * sin(a * degree), D10]);
            ba = append(ba, [armP(RM[0] + 0.6875 * cos(a * degree), RM[1] + 0.6875 * sin(a * degree), 11.425), vector(0, -1, 0), 0.375]);
        }
        plate(context, aid, "rmp", "IN-11 Roller motor plate 1/8 6061", armCS(11.425), [[RM[0] - 1.2, RM[1] - 1.2], [RM[0] + 1.2, RM[1] - 1.2], [RM[0] + 1.2, RM[1] + 1.2], [RM[0] - 1.2, RM[1] + 1.2]], 0.125,
            { "r" : 0.3, "holes" : rmh }, C_ALBK, "Aluminum 6061");
        for (var i = 0; i < 4; i += 1)
        {
            const a = (45 + 90 * i) * degree;
            spacer(context, aid, "rms" ~ i, "IN-12 Motor plate standoff 0.375 OD x 1.125", armP(RM[0] + 0.9 * cos(a), RM[1] + 0.9 * sin(a), 11.425), armP(RM[0] + 0.9 * cos(a), RM[1] + 0.9 * sin(a), IYP0), 0.1875, 0.08, C_AL);
            ba = append(ba, [armP(RM[0] + 0.9 * cos(a), RM[1] + 0.9 * sin(a), IYP1), vector(0, -1, 0), 1.5, false, true]);       // flush (stowed, the tower is 0.075 away)
        }
        htd(context, aid, "pm", armP(RM[0], RM[1], yA), vector(0, 1, 0), 12, 0.354, 0.315);
        htd(context, aid, "pt2a", armP(T2[0], T2[1], yA), vector(0, 1, 0), 28, 0.354, 0.5);
        htd(context, aid, "pt2b", armP(T2[0], T2[1], yB), vector(0, 1, 0), 24, 0.354, 0.5);
        htd(context, aid, "pt1b", armP(T1[0], T1[1], yB), vector(0, 1, 0), 24, 0.354, 0.5);
        htd(context, aid, "pt1a", armP(T1[0], T1[1], yA), vector(0, 1, 0), 24, 0.354, 0.5);
        htd(context, aid, "pfr", armP(IL, 0, yA), vector(0, 1, 0), 24, 0.354, 0.5);
        const rp12 = 12 * 5 / PI / 2 / 25.4;
        const rp24 = 24 * 5 / PI / 2 / 25.4;
        const rp28 = 28 * 5 / PI / 2 / 25.4;
        beltLoop(context, aid, "bt1", "IN-13 Roller belt 9 mm HTD (motor -> rear top)", armP(RM[0], RM[1], yA), rp12, armP(T2[0], T2[1], yA), rp28, vector(0, 1, 0), 0.354);
        beltLoop(context, aid, "bt2", "IN-14 Roller belt 9 mm HTD (rear top -> front top)", armP(T2[0], T2[1], yB), rp24, armP(T1[0], T1[1], yB), rp24, vector(0, 1, 0), 0.354);
        beltLoop(context, aid, "bt3", "IN-15 Roller belt 9 mm HTD (front top -> front roller)", armP(T1[0], T1[1], yA), rp24, armP(IL, 0, yA), rp24, vector(0, 1, 0), 0.354);

        // deploy pulley on the pivot, outside the right tower
        htd(context, aid, "pdep", armP(0, 0, -13.45), vector(0, 1, 0), 36, 0.354, 0.5);

        // roller X44 lead: off the motor's back end, over the arm's top edge, down its front edge to the
        // pivot service loop (meets WR-30 on the frame)
        cable(context, aid, "wr", "IN-30 Roller X44 harness (arm run)", [armP(RM[0], RM[1], 8.2) / inch, armP(RM[0], 9.8, 8.2) / inch, armP(RM[0], 9.8, 12.2) / inch,
                armP(2.6, 9.6, 12.2) / inch, armP(1.5, 7.7, 12.2) / inch, armP(1.05, 1.6, 12.2) / inch, armP(0.3, -0.75, 12.2) / inch], 0.24, C_HARN);

        bolts(context, aid, "hw", ba);
        if (!definition.deployed)
            spinAxis(context, id + "fold", qCreatedBy(aid, EntityType.BODY), P(IOX, 0, IOZ), vector(0, 1, 0), -127 * degree);

        // ================= fixed: towers, pivot bearings, deploy drive =================
        // towers run back to the mid uprights: they are also the hopper's side liners
        const tow = [[0.05, 9.3], [9.4, 9.3], [9.6, 6.45], [12.4, 6.45], [13.4, 7.3], [13.4, 11.3], [12.3, 20.95], [0.05, 20.95]];
        const dgx = 11.2;
        const dgz = 7.4;
        for (var sy in [1, -1])
        {
            var holes = [[IOX, IOZ, 1.125]];
            for (var x in [0.525, 2.025, 4.025, 6.525, 9.025, 11.525])
                holes = append(holes, [x, 20.5, D10]);
            if (sy > 0)
            {
                for (var x in [1.1, 1.9])
                    holes = append(holes, [x, 19.65, D10]);                  // hopper CANrange bracket
                holes = append(holes, [2.6, 19.2, 0.3]);                     // its CAN lead
            }
            for (var z in [10.125, 14.125, 18.125])
                holes = append(holes, [3.75, z, D10]);
            if (sy < 0)
            {
                holes = append(holes, [dgx, dgz, 0.6]);
                for (var i = 0; i < 4; i += 1)
                    holes = append(holes, [dgx + 0.75 * cos((45 + 90 * i) * degree), dgz + 0.75 * sin((45 + 90 * i) * degree), D10]);
            }
            var bos = holeBosses(holes, 0.3);
            bos = append(bos, [IOX, IOZ, 1.0]);
            bos = append(bos, [dgx, dgz, 1.0]);
            const pk = autoTruss(tow, bos, 1.8, 0.3, 0.35);
            plate(context, id, "tw" ~ (sy > 0 ? "L" : "R"), "IN-2" ~ (sy > 0 ? 0 : 1) ~ " Intake tower / hopper liner " ~ (sy > 0 ? "L" : "R") ~ " 1/8 6061",
                csXZ(sy > 0 ? 13.0 : -12.875), tow, 0.125, { "r" : 0.25, "holes" : holes, "pockets" : pk, "rp" : 0.1 }, C_ALBK, "Aluminum 6061");
            fbearing(context, id, "tb" ~ (sy > 0 ? "L" : "R"), P(IOX, sy * 12.971, IOZ), vector(0, -sy, 0), 1.125, 0.3125, 0.5, 0);
            for (var x in [0.525, 2.025, 4.025, 6.525, 9.025, 11.525])
                bf = append(bf, [P(x, sy * 12.875, 20.5), vector(0, sy, 0), 0.25]);
            for (var z in [10.125, 14.125, 18.125])
                bf = append(bf, [P(3.75, sy * 12.875, z), vector(0, sy, 0), 0.25]);
        }
        // deploy: X44 + 25:1 MAXPlanetary bolted to the inside of the right tower, output through it
        turned(context, id, "dgb", "IN-22 REV MAXPlanetary 5:1 + 5:1 (deploy)", P(dgx, -12.875, dgz), vector(0, 1, 0),
            [[0, 0.3], [0, 0.95], [0.15, 0.95], [0.15, 0.88], [2.7, 0.88], [2.7, 0.4], [2.45, 0.4], [2.45, 0.3]], 0, 0, C_BLK, "Aluminum 6061");     // input counterbore for the X44 pilot
        kraken(context, id + "dmot", P(dgx, -12.875 + 2.7, dgz), vector(0, -1, 0), false);
        hexShaft(context, id, "dsh", "IN-23 Deploy output shaft 1/2 hex", P(dgx, -13.75, dgz), P(dgx, -12.6, dgz), 0.5, C_ST, "Steel");
        htd(context, id, "pdg", P(dgx, -13.45, dgz), vector(0, 1, 0), 12, 0.354, 0.5);
        beltLoop(context, id, "dbelt", "IN-24 Deploy belt 9 mm HTD 5 mm (12T:36T)", P(dgx, -13.45, dgz), rp12, P(IOX, -13.45, IOZ), 36 * 5 / PI / 2 / 25.4, vector(0, 1, 0), 0.354);
        for (var i = 0; i < 4; i += 1)
            bf = append(bf, [P(dgx + 0.75 * cos((45 + 90 * i) * degree), -13.0, dgz + 0.75 * sin((45 + 90 * i) * degree)), vector(0, 1, 0), 0.375]);
        boltsIn(context, id, "hw", bf, qSubtraction(qCreatedBy(id, EntityType.BODY), qCreatedBy(aid, EntityType.BODY)));
    });
