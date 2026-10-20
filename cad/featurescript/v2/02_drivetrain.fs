
// =========================================================================
// 01 DRIVETRAIN: 4x SDS MK5n (vendor CAD) with Kraken X60 drive / X44 steer,
// 2x1x1/16 perimeter (front/rear full width, sides between), 1x1x1/16 cross
// members, 1/8 6061 pocketed bellypan, hardware.
// =========================================================================
// module placement: [A or B, rotation deg, corner x, corner y]
function moduleCorners() returns array
{
    return [[true, 0, FX, FY], [false, 0, FX, -FY], [false, 180, -FX, FY], [true, 180, -FX, -FY]];
}

function moduleT(m is array) returns Transform
{
    return transform(rotationMatrix3d(vector(0, 0, 1), m[1] * degree), P(m[2], m[3], 0));
}

// true when no bellypan keep-out point lies within 0.6 in of the rectangle
function keepClear(u0 is number, v0 is number, u1 is number, v1 is number) returns boolean
{
    for (var p in bellyKeep())
    {
        const dx = max(0, max(u0 - p[0], p[0] - u1));
        const dy = max(0, max(v0 - p[1], p[1] - v1));
        if (dx * dx + dy * dy < 0.36)
            return false;
    }
    return true;
}

// bellypan bolt locations: on the 0.25 / 0.75 grid of the tube hole patterns so every bolt
// passes through an existing tube hole into a rivnut
function bellyHoles() returns array
{
    var h = [];
    for (var y in [-7.25, -5.25, -3.25, -1.25, 1.25, 3.25, 5.25, 7.25])
    {
        h = append(h, [FX - 0.5, y]);
        h = append(h, [-FX + 0.5, y]);
    }
    for (var x in [-5.25, -3.25, -1.25, 1.25, 3.25, 5.25])
    {
        h = append(h, [x, FY - 0.5]);
        h = append(h, [x, -FY + 0.5]);
    }
    for (var y in [-12.25, -8.25, -4.25, 4.25, 8.25, 12.25])
    {
        h = append(h, [5.1, y]);
        h = append(h, [-5.1, y]);
    }
    return h;
}

// vendor bodies come in as "Part n": give them BOM names from their shape and corner
function nameModule(context is Context, q is Query)
{
    const all = evBox3d(context, { "topology" : q, "tight" : true });
    const cx = (all.minCorner[0] + all.maxCorner[0]) / 2;
    const cy = (all.minCorner[1] + all.maxCorner[1]) / 2;
    const corner = (cx > 0 ? "front " : "rear ") ~ (cy > 0 ? "left" : "right");
    for (var b in evaluateQuery(context, q))
    {
        const bb = evBox3d(context, { "topology" : b, "tight" : true });
        const z0 = bb.minCorner[2] / inch;
        const dz = (bb.maxCorner[2] - bb.minCorner[2]) / inch;
        const w = max((bb.maxCorner[0] - bb.minCorner[0]) / inch, (bb.maxCorner[1] - bb.minCorner[1]) / inch);
        var nm = "SDS MK5n ";
        if (z0 < 0.05)
            nm = nm ~ "4 in wheel, billet hub + TPU tread";
        else if (w < 0.6 && dz > 2.5)
            nm = nm ~ "frame standoff";
        else if (w < 0.6)
            nm = nm ~ "frame bolt";
        else if (dz < 0.5)
            nm = nm ~ "bottom plate";
        else if (z0 > 3)
            nm = nm ~ "top plate + steering housing";
        else
            nm = nm ~ "fork + drive gearing";
        setProperty(context, { "entities" : b, "propertyType" : PropertyType.NAME, "value" : nm ~ " (" ~ corner ~ ")" });
    }
}

annotation { "Feature Type Name" : "APEX 01 Drivetrain" }
export const apexDrivetrain = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        // ---- swerve modules (SDS release geometry) + motors
        var i = 0;
        for (var m in moduleCorners())
        {
            i += 1;
            const T = moduleT(m);
            cots(context, id + ("mk" ~ i), m[0] ? MK5NA::build : MK5NB::build, T, "SDS MK5n module", 3.9);
            nameModule(context, qCreatedBy(id + ("mk" ~ i), EntityType.BODY));
            // motors hang under the inboard end of the housing, shafts up into it
            const sy = m[0] ? -1 : 1;     // A modules sit in the -Y quadrant of their corner frame
            // mounting faces mate the underside of the module plate (z 4.594), pilots in its bores
            kraken(context, id + ("dm" ~ i), T * P(-6.32, sy * 2.18, 4.594), T.linear * vector(0, 0, 1), true);
            kraken(context, id + ("sm" ~ i), T * P(-5.69, sy * 4.13, 4.594), T.linear * vector(0, 0, 1), false);
        }

        // ---- perimeter: 2x1x1/16, 2 in faces vertical, #10 holes on 0.5 in pitch
        tube(context, id, "tF", "DT-01 Front rail 2x1x1/16 (28.00 in)", P(FX - 1, -FY, RZ0), P(FX, FY, RZ1), TW, true, C_ALBK);
        tube(context, id, "tB", "DT-02 Rear rail 2x1x1/16 (28.00 in)", P(-FX, -FY, RZ0), P(-FX + 1, FY, RZ1), TW, true, C_ALBK);
        tube(context, id, "tL", "DT-03 Left rail 2x1x1/16 (25.00 in)", P(-FX + 1, FY - 1, RZ0), P(FX - 1, FY, RZ1), TW, true, C_ALBK);
        tube(context, id, "tR", "DT-04 Right rail 2x1x1/16 (25.00 in)", P(-FX + 1, -FY, RZ0), P(FX - 1, -FY + 1, RZ1), TW, true, C_ALBK);
        // cross members: battery / electronics bay boundary and rotor-motor support
        tube(context, id, "tX1", "DT-05 Cross member 1x1x1/16 (26.00 in)", P(-5.6, -FY + 1, RZ0), P(-4.6, FY - 1, RZ0 + 1), TW, true, C_ALBK);
        tube(context, id, "tX2", "DT-06 Cross member 1x1x1/16 (26.00 in)", P(4.6, -FY + 1, RZ0), P(5.6, FY - 1, RZ0 + 1), TW, true, C_ALBK);

        // ---- bellypan: 1/8 6061 under the frame, notched around the module bottom plates
        {
            // corner notches clear the module wheel and both inverted motors (7.6 x 6.3 in)
            const F = MKFOOT + 0.05;
            const G = MKLONG + 0.12;
            const outline = [[FX - G, -FY], [FX - G, -FY + F], [FX, -FY + F], [FX, FY - F], [FX - G, FY - F], [FX - G, FY],
                    [-FX + G, FY], [-FX + G, FY - F], [-FX, FY - F], [-FX, -FY + F], [-FX + G, -FY + F], [-FX + G, -FY]];
            var pockets = [];
            // pocket grid: between the cross members (electronics zone left solid) and outboard of them
            for (var x in [-3.675, -1.025, 1.625])
                for (var y = -11.625; y < 11.0; y += 2.65)
                {
                    const solid = (x > -3.2 && x < 3.2 && y > -7.0 && y < 7.0);   // roboRIO / PDH / CANivore plate area
                    if (!solid && keepClear(x, y, x + 2.05, y + 2.05))
                        pockets = append(pockets, rrect(x, y, x + 2.05, y + 2.05, 0.35));
                }
            for (var x in [-8.25, 6.2])
                for (var y = -6.325; y < 5.0; y += 2.65)
                    if (keepClear(x, y, x + 2.05, y + 2.05))
                        pockets = append(pockets, rrect(x, y, x + 2.05, y + 2.05, 0.35));
            var holes = [];
            for (var h in bellyHoles())
                holes = append(holes, [h[0], h[1], D10]);
            for (var h in bellyBolts())
                holes = append(holes, [h[0], h[1], D10]);
            plate(context, id, "belly", "DT-07 Bellypan 1/8 6061 (pocketed)", csXY(BPZ), outline, 0.125,
                { "r" : 0.25, "holes" : holes, "pockets" : pockets, "rp" : 0.35 }, C_AL, "Aluminum 6061");
        }

        // ---- hardware: bellypan to frame rails and cross members (every bellypan bolt hole)
        var bl = [];
        for (var h in bellyHoles())
            bl = append(bl, [P(h[0], h[1], BPZ), vector(0, 0, 1), 0.5]);
        bolts(context, id, "hw", bl);
    });
