
// =========================================================================
// 02 BUMPERS: 3/4 in plywood backing (four boards, 45 deg mitred, outer
// corners relieved for the cover radius), 2.5 in pool-noodle padding under a
// fabric cover swept around the frame (filled corners), 5.0 in tall (Z 1.75 -
// 6.75 covers the BUMPER ZONE 2.5 - 5.75 per R405), 3.25 in deep (R403 <= 4.0),
// one 7.8 in rear gap for battery removal (R401), team numbers front + sides
// (R412), 1x1x1/8 angle clips.
// =========================================================================
const BZ0 = 1.75;
const BZ1 = 6.75;
const BB = 0.75;            // backing thickness
const BP = 2.5;             // padding + cover depth beyond backing
const BRC = 0.9;            // cover path corner radius at the backing face

// bumper cover cross-section (u outward from r0, v = z): square inner edge, rounded outer corners
function coverProfile(r0 is number) returns array
{
    const r = 0.45;
    var pts = [[r0, BZ0]];
    for (var i = 0; i <= 6; i += 1)
    {
        const a = (-90 + 90 * i / 6) * degree;
        pts = append(pts, [r0 + BP - r + r * cos(a), BZ0 + r + r * sin(a)]);
    }
    for (var i = 0; i <= 6; i += 1)
    {
        const a = (90 * i / 6) * degree;
        pts = append(pts, [r0 + BP - r + r * cos(a), BZ1 - r + r * sin(a)]);
    }
    pts = append(pts, [r0, BZ1]);
    return pts;
}

annotation { "Feature Type Name" : "APEX 02 Bumpers" }
export const apexBumpers = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Red alliance" }
        definition.red is boolean;
        annotation { "Name" : "Team number", "Default" : "9999" }
        definition.teamNumber is string;
    }
    {
        const col = definition.red ? C_BUMPR : C_BUMPB;
        const ox = FX + BB;
        const oy = FY + BB;
        const m = BRC / 2;

        // ---- backing boards (plan-view outlines, extruded Z BZ0 -> BZ1)
        plate(context, id, "bkF", "BU-01 Front backing 3/4 plywood", csXY(BZ0),
            [[FX, -FY], [FX, FY], [ox - m, oy - m], [ox, oy - BRC], [ox, -oy + BRC], [ox - m, -oy + m]], BZ1 - BZ0, { "r" : 0 }, C_WOOD, "Plywood");
        // rear: two segments either side of the battery gap (R401: one gap > 1.25 in, >= 5 in of BUMPER each side of each corner)
        plate(context, id, "bkB", "BU-02 Rear backing R 3/4 plywood", csXY(BZ0),
            [[-FX, BATGAP0], [-FX, -FY], [-ox + m, -oy + m], [-ox, -oy + BRC], [-ox, BATGAP0]], BZ1 - BZ0, { "r" : 0 }, C_WOOD, "Plywood");
        plate(context, id, "bkB2", "BU-02 Rear backing L 3/4 plywood", csXY(BZ0),
            [[-FX, FY], [-FX, BATGAP1], [-ox, BATGAP1], [-ox, oy - BRC], [-ox + m, oy - m]], BZ1 - BZ0, { "r" : 0 }, C_WOOD, "Plywood");
        plate(context, id, "bkL", "BU-03 Left backing 3/4 plywood", csXY(BZ0),
            [[-FX, FY], [FX, FY], [ox - m, oy - m], [ox - BRC, oy], [-ox + BRC, oy], [-ox + m, oy - m]], BZ1 - BZ0, { "r" : 0 }, C_WOOD, "Plywood");
        plate(context, id, "bkR", "BU-04 Right backing 3/4 plywood", csXY(BZ0),
            [[FX, -FY], [-FX, -FY], [-ox + m, -oy + m], [-ox + BRC, -oy], [ox - BRC, -oy], [ox - m, -oy + m]], BZ1 - BZ0, { "r" : 0 }, C_WOOD, "Plywood");

        // ---- padding + fabric: four straight runs + four revolved corners, united
        {
            const pid = id + "cover";
            const c = new box(0);
            // straight runs: cs x = outward normal, cs z = run direction, cs y = up
            for (var n in [vector(1, 0, 0), vector(0, 1, 0), vector(-1, 0, 0), vector(0, -1, 0)])
            {
                const d = vector(n[1], -n[0], 0);
                const half = abs(n[0]) > 0.5 ? oy - BRC : ox - BRC;
                const face = abs(n[0]) > 0.5 ? ox : oy;
                const o = (n * face - d * half) * inch;
                if (n[0] < -0.5)
                {
                    // rear run (d = +Y): split at the battery gap
                    prism(context, nextId(pid, c), coordSystem(o, n, d), coverProfile(0), 0, BATGAP0 + half);
                    prism(context, nextId(pid, c), coordSystem(o, n, d), coverProfile(0), BATGAP1 + half, 2 * half);
                }
                else
                    prism(context, nextId(pid, c), coordSystem(o, n, d), coverProfile(0), 0, 2 * half);
            }
            // corners: profile on the radial plane, revolved 90 deg CCW about the corner axis
            const cx = [ox - BRC, -ox + BRC, -ox + BRC, ox - BRC];
            const cy = [oy - BRC, oy - BRC, -oy + BRC, -oy + BRC];
            for (var k = 0; k < 4; k += 1)
            {
                const u = vector(cos(90 * k * degree), sin(90 * k * degree), 0);
                const sid = nextId(pid, c);
                const sk = newSketchOnPlane(context, sid + "sk", { "sketchPlane" : plane(P(cx[k], cy[k], 0), cross(u, vector(0, 0, 1)), u) });
                var pts = [];
                for (var p in coverProfile(BRC))
                    pts = append(pts, vector(p[0], p[1]) * inch);
                pts = append(pts, pts[0]);
                skPolyline(sk, "pr", { "points" : pts });
                skSolve(sk);
                opRevolve(context, sid + "rv", { "entities" : qSketchRegion(sid + "sk"), "axis" : line(P(cx[k], cy[k], 0), vector(0, 0, 1)), "angleForward" : 90 * degree });
                opDeleteBodies(context, sid + "del", { "entities" : qCreatedBy(sid + "sk", EntityType.BODY) });
            }
            done(context, pid, pid + "h", "BU-05 Padding (2.5 in pool noodle) + fabric cover", col, "Foam");
        }

        // ---- team numbers (white, raised 0.03 in) on front and both sides: 3 locations 90 deg apart (R412);
        //      the rear is split by the battery gap
        const faces = [[P(ox + BP + 0.03, 0, 0), vector(1, 0, 0), vector(0, 1, 0)],
                [P(0, oy + BP + 0.03, 0), vector(0, 1, 0), vector(-1, 0, 0)],
                [P(0, -oy - BP - 0.03, 0), vector(0, -1, 0), vector(1, 0, 0)]];
        var fi = 0;
        for (var f in faces)
        {
            fi += 1;
            const tid = id + ("num" ~ fi);
            try silent
            {
                const sk = newSketchOnPlane(context, tid + "sk", { "sketchPlane" : plane(f[0], f[1], f[2]) });
                skText(sk, "t", { "text" : definition.teamNumber, "fontName" : "OpenSans-Bold.ttf",
                            "firstCorner" : vector(-5.0, BZ0 + 0.55) * inch, "secondCorner" : vector(5.0, BZ1 - 0.55) * inch });
                skSolve(sk);
                opExtrude(context, tid + "ex", { "entities" : qSketchRegion(tid + "sk", true), "direction" : -f[1], "endBound" : BoundingType.BLIND, "endDepth" : 0.03 * inch });     // 0.03 proud of the cover
                opDeleteBodies(context, tid + "del", { "entities" : qCreatedBy(tid + "sk", EntityType.BODY) });
                const q = qCreatedBy(tid + "ex", EntityType.BODY);
                setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : "BU-10 Team number (heat-transfer vinyl)" });
                setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : C_WHT });
                setProperty(context, { "entities" : q, "propertyType" : PropertyType.MATERIAL, "value" : mat("Fabric") });
            }
        }

        // ---- mounting: 1x1x1/8 6061 angle clips (horizontal leg on the rail, vertical
        //      leg against the backing above the rail), 8 total, between the modules
        var mi = 0;
        var bl = [];
        const t = 0.125;
        for (var s in [1, -1])
            for (var y in [-4.75, 4.75])
            {
                mi += 1;
                const x0 = s * FX;
                const L = [[x0 - s, RZ1], [x0, RZ1], [x0, RZ1 + 1], [x0 - s * t, RZ1 + 1], [x0 - s * t, RZ1 + t], [x0 - s, RZ1 + t]];
                // rear left clip squeezed between the battery gap and the rear deck post
                const narrow = s < 0 && y > 0;
                const y1 = narrow ? 6.05 : y + 1.0;
                const w = narrow ? 1.15 : 2.0;
                const yb = narrow ? 5.25 : y;
                plate(context, id, "clip" ~ mi, "BU-20 Bumper clip 1x1x1/8 angle x " ~ (narrow ? "1.15" : "2") ~ " in", csXZ(y1), L, w, { "r" : 0 }, C_AL, "Aluminum 6061");
                bl = append(bl, [P(x0 - s * 0.5, yb, RZ1 + t), vector(0, 0, -1), 0.375]);     // into a rivnut in the rail
                bl = append(bl, [P(x0 - s * t, y1 - w / 2, RZ1 + 0.6), vector(s, 0, 0), 0.875]);
            }
        for (var s in [1, -1])
            for (var x in [-3.25, 1.25])
            {
                mi += 1;
                const y0 = s * FY;
                const L = [[y0 - s, RZ1], [y0, RZ1], [y0, RZ1 + 1], [y0 - s * t, RZ1 + 1], [y0 - s * t, RZ1 + t], [y0 - s, RZ1 + t]];
                plate(context, id, "clip" ~ mi, "BU-20 Bumper clip 1x1x1/8 angle x 2 in", csYZ(x - 1.0), L, 2.0, { "r" : 0 }, C_AL, "Aluminum 6061");
                bl = append(bl, [P(x, y0 - s * 0.5, RZ1 + t), vector(0, 0, -1), 0.375]);
                bl = append(bl, [P(x, y0 - s * t, RZ1 + 0.6), vector(0, s, 0), 0.875]);
            }
        bolts(context, id, "hw", bl);
    });
