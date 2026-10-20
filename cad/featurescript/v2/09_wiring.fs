
// =========================================================================
// 08 WIRING (fixed robot). Routed harness modelled as sleeved bundles with
// branch tails, every run clipped to structure:
//  - 6 AWG: battery -> SB50 -> 120 A breaker -> PDH (+), SB50 -> PDH (-)
//  - left PDH output bundle -> board pass-through -> left side trunk ->
//    FL / BL module tails + intake roller (to the arm service loop)
//  - inner PDH bundle -> board-edge drop -> belly trunk T1 (in front of the
//    rear cross member) -> right side trunk -> FR / BR tails, intake deploy,
//    rotor X60; yaw X44 and the right riser (roboRIO power, CAN, MPM) off T1
//  - uptake X44 straight from the PDH; turret harness up through the deck
//    grommet outside the wrap ring and in through the ring wall to the
//    service loop's fixed end
//  - Ethernet / 12 V network bundle up the right side to the switch; one
//    rear deck run (under the wrap ring) to the radio and both rear
//    Limelights; turret Cat6 breaks out at the grommet
//  - CAN to the Pigeon and both CANranges, USB roboRIO -> CANivore, RSL lead
// The turret and intake-arm runs are built in those (moving) features.
// =========================================================================
const C_HARN = color(0.12, 0.12, 0.13);     // black braided sleeve
const C_SIG = color(0.95, 0.75, 0.10);      // CAN yellow
const C_6R = color(0.80, 0.10, 0.10);       // 6 AWG red
const C_6B = color(0.08, 0.08, 0.09);       // 6 AWG black

// round cable along a polyline (inches): segments + spheres at the bends
function cable(context is Context, id is Id, k is string, name is string, pts is array, d is number, col is Color)
{
    const pid = id + k;
    const c = new box(0);
    const n = size(pts);
    for (var i = 0; i < n - 1; i += 1)
        sCyl(context, pid, c, vin(pts[i]), vin(pts[i + 1]), d / 2);
    for (var i = 1; i < n - 1; i += 1)
        { opPoint(context, pid + ("q" ~ i), { "point" : vin(pts[i]) }); fSphere(context, nextId(pid, c), { "center" : qCreatedBy(pid + ("q" ~ i), EntityType.VERTEX), "radius" : d / 2 * inch }); opDeleteBodies(context, pid + ("x" ~ i), { "entities" : qCreatedBy(pid + ("q" ~ i), EntityType.BODY) }); }
    done(context, pid, id + (k ~ "h"), name, col, "Cable");
}

function mirrorY(pts is array) returns array
{
    var out = [];
    for (var p in pts)
        out = append(out, [p[0], -p[1], p[2]]);
    return out;
}

annotation { "Feature Type Name" : "APEX 08 Wiring" }
export const apexWiring = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        // ================= main power, 6 AWG =================
        cable(context, id, "p1", "WR-01 Battery lead + 6 AWG", [[-11.95, -2.08, 10.2], [-11.95, -2.08, 10.75], [-11.98, -1.6, 11.02], [-11.98, -1.35, 11.02]], 0.32, C_6R);
        cable(context, id, "p2", "WR-02 Battery lead - 6 AWG", [[-11.95, 3.98, 10.2], [-11.95, 3.98, 10.75], [-12.95, 3.98, 10.8], [-12.95, -1.75, 10.9], [-12.42, -1.75, 11.02], [-12.42, -1.35, 11.02]], 0.32, C_6B);
        cable(context, id, "p3", "WR-03 SB50 -> main breaker + 6 AWG", [[-11.98, 2.07, 11.02], [-11.9, 2.2, 11.4], [-11.9, 2.2, 13.6], [-11.9, 0.95, 13.6]], 0.32, C_6R);
        cable(context, id, "p4", "WR-04 Main breaker -> PDH + 6 AWG", [[-11.9, -0.95, 13.6], [-11.9, -2.2, 13.6], [-11.9, -2.2, 12.2], [-10.9, -2.2, 12.1], [-10.9, 7.4, 11.8], [-11.2, 9.6, 10.2], [-11.73, 10.1, 8.75]], 0.32, C_6R);
        cable(context, id, "p5", "WR-05 SB50 -> PDH - 6 AWG", [[-12.42, 1.95, 11.02], [-12.42, 2.4, 11.1], [-12.2, 6.0, 11.2], [-12.2, 7.4, 11.2], [-12.0, 8.3, 9.3], [-11.73, 9.3, 8.75]], 0.32, C_6B);

        // ================= motor harnesses (12 AWG pairs + CAN, sleeved) =================
        cable(context, id, "b1", "WR-10 PDH outer-row bundle (left motors)", [[-10.5, 13.3, 7.3], [-2.5, 13.3, 7.3], [-2.5, 12.6, 6.55], [-2.5, 12.6, 3.35], [-2.5, 12.3, 3.0]], 0.45, C_HARN);
        cable(context, id, "b2", "WR-11 Left side trunk", [[-5.8, 12.3, 3.0], [5.6, 12.3, 3.0]], 0.45, C_HARN);
        cable(context, id, "b3", "WR-12 PDH inner-row bundle (crossover)", [[-10.5, 8.2, 7.3], [-4.0, 8.2, 7.3], [-4.0, 7.5, 7.3], [-4.0, 7.5, 2.3]], 0.5, C_HARN);
        cable(context, id, "t1", "WR-13 Belly trunk T1", [[-4.0, 7.5, 2.3], [-4.0, -12.3, 2.3], [-4.0, -12.3, 3.0]], 0.5, C_HARN);
        cable(context, id, "b4", "WR-14 Right side trunk", [[-5.8, -12.3, 3.0], [5.6, -12.3, 3.0]], 0.45, C_HARN);
        // module tails
        const t60 = [[5.6, 12.3, 3.0], [5.85, 11.82, 2.6]];
        const t44 = [[5.5, 12.3, 3.0], [5.78, 11.3, 3.35], [5.78, 9.87, 3.1], [6.8, 9.87, 2.6]];
        const b60 = [[-5.8, 12.3, 3.0], [-5.85, 11.82, 2.6]];
        const b44 = [[-5.7, 12.3, 3.0], [-5.78, 11.3, 3.35], [-5.78, 9.87, 3.1], [-6.8, 9.87, 2.6]];
        cable(context, id, "fl60", "WR-20 FL drive X60 tail", t60, 0.24, C_HARN);
        cable(context, id, "fl44", "WR-21 FL steer X44 tail", t44, 0.24, C_HARN);
        cable(context, id, "bl60", "WR-22 BL drive X60 tail", b60, 0.24, C_HARN);
        cable(context, id, "bl44", "WR-23 BL steer X44 tail", b44, 0.24, C_HARN);
        cable(context, id, "fr60", "WR-24 FR drive X60 tail", mirrorY(t60), 0.24, C_HARN);
        cable(context, id, "fr44", "WR-25 FR steer X44 tail", mirrorY(t44), 0.24, C_HARN);
        cable(context, id, "br60", "WR-26 BR drive X60 tail", mirrorY(b60), 0.24, C_HARN);
        cable(context, id, "br44", "WR-27 BR steer X44 tail", mirrorY(b44), 0.24, C_HARN);
        // mechanisms
        cable(context, id, "inr", "WR-30 Intake roller X44 harness (to arm service loop)",
            [[4.4, 12.3, 3.0], [4.4, 7.4, 3.4], [6.2, 7.4, 3.5], [11.0, 7.4, 4.3], [12.0, 7.4, 6.35], [12.3, 11.9, 6.35], [12.3, 12.2, 6.6], [12.3, 12.2, 8.8], [12.8, 12.2, 9.15]], 0.24, C_HARN);
        cable(context, id, "ind", "WR-31 Intake deploy X44 harness",
            [[4.4, -12.3, 3.0], [4.4, -7.4, 3.4], [6.2, -7.4, 3.5], [10.0, -7.4, 6.2], [10.8, -7.0, 6.9], [11.2, -7.0, 7.4]], 0.24, C_HARN);
        cable(context, id, "rot", "WR-32 Serializer X60 harness", [[0.7, -12.3, 3.2], [0.7, -12.3, 7.4], [0.7, -11.62, 8.3]], 0.24, C_HARN);
        cable(context, id, "yaw", "WR-33 Turret yaw X44 harness", [[-4.0, -3.79, 2.5], [-4.3, -3.79, 2.95], [-5.1, -3.79, 3.05], [-9.9, -3.79, 3.05], [-9.9, -3.79, 7.9]], 0.24, C_HARN);
        cable(context, id, "upt", "WR-34 Uptake X44 harness", [[-8.25, 8.2, 7.55], [-8.25, 7.6, 9.6], [-8.25, 6.9, 11.0]], 0.24, C_HARN);
        // up through the deck grommet outside the wrap ring, in through the ring wall to the loop's fixed end
        cable(context, id, "tur", "WR-35 Turret harness (2x X60, X44, CAN, Cat6) to the cable wrap",
            [[-5.5, 8.2, 7.55], [-6.2, 8.25, 8.6], [TGROMX, TGROMY, 9.4], [TGROMX, TGROMY, WRAPZ],
                [TURX + WRAPRO * cos(WRAPAS * degree), WRAPRO * sin(WRAPAS * degree), WRAPZ]], 0.3, C_HARN);

        // ================= right riser: roboRIO power, CAN, MPM feed =================
        cable(context, id, "rr", "WR-40 Right riser (roboRIO power, CAN buses, MPM feed)", [[-4.0, -7.45, 2.5], [-4.0, -7.45, 6.6], [-4.0, -7.6, 7.3], [-6.2, -7.6, 7.3], [-6.5, -7.75, 7.6]], 0.3, C_HARN);
        cable(context, id, "cnv", "WR-41 CANivore CAN FD (to drivetrain)", [[-2.25, -10.95, 7.25], [-2.35, -8.5, 7.25], [-2.35, -7.9, 7.25], [-3.8, -7.6, 7.3]], 0.12, C_SIG);
        cable(context, id, "usb", "WR-42 USB roboRIO -> CANivore", [[-9.5, -13.7, 7.5], [-8.0, -13.7, 7.25], [-3.3, -13.7, 7.25], [-3.1, -13.3, 7.25]], 0.15, C_BLK);
        cable(context, id, "rsl", "WR-43 RSL lead (roboRIO)", [[-11.0, -7.85, 7.6], [-11.2, -7.6, 9.5], [RSLX, RSLY, 11.55]], 0.12, C_BLK);
        cable(context, id, "pig", "WR-44 Pigeon 2 CAN FD", [[-3.75, 3.0, 2.3], [-3.54, 3.0, 2.05]], 0.12, C_SIG);
        cable(context, id, "cr1", "WR-45 CANrange (uptake) CAN (to the uptake X44 tail)", [[-7.4, 2.2, 10.65], [-6.9, 2.2, 10.75], [-6.9, 7.3, 10.0], [-8.1, 7.6, 9.6]], 0.12, C_SIG);
        cable(context, id, "cr2", "WR-46 CANrange (hopper) CAN", [[2.3, 12.1, 19.55], [2.6, 12.6, 19.25], [2.6, 13.6, 19.05], [2.6, 13.6, 4.0], [2.1, 12.85, 4.0], [2.1, 12.55, 3.15]], 0.12, C_SIG);

        // ================= network + 12 V (MPM) to the deck =================
        cable(context, id, "net", "WR-50 Network + 12 V bundle (roboRIO, MPM -> switch)",
            [[-7.5, -13.2, 8.6], [-7.5, -13.45, 9.0], [-7.5, -13.45, 13.55], [-7.5, -12.45, 13.6]], 0.3, C_ETH);
        // rear deck run (under the wrap ring floor): switch -> radio (Ethernet + 12 V), rear LL4s, turret Cat6
        cable(context, id, "e3", "WR-53 Rear deck network run (switch -> radio, cameras)",
            [[-9.05, -10.9, 13.6], [-9.6, -9.2, 13.3], [-13.1, -9.2, 13.3], [-13.1, 9.1, 13.3], [-9.6, 9.1, 13.3], [-9.05, 10.6, 13.6]], 0.3, C_ETH);
        cable(context, id, "e1", "WR-51 Ethernet rear-left LL4", [[-10.45, 10.9, 13.45], [-10.3, 9.5, 13.3], [-10.0, 9.1, 13.3]], 0.22, C_ETH);
        cable(context, id, "e2", "WR-52 Ethernet rear-right LL4", [[-10.45, -10.9, 13.45], [-10.3, -9.5, 13.3], [-10.0, -9.2, 13.3]], 0.22, C_ETH);
        cable(context, id, "e4", "WR-54 Ethernet turret LL4 (out of the turret harness)", [[TGROMX, TGROMY, 13.45], [-9.6, 8.7, 13.3], [-10.0, 9.1, 13.3]], 0.22, C_ETH);
    });
