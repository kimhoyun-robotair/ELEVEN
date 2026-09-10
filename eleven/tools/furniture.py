"""Original, metre-scale furniture for the multi-floor Blender generators.

Every public builder accepts ``C, parent, x, y, z, rot=0``.  ``z`` is the
finished floor elevation, ``rot`` is a yaw in radians, and the returned Empty
contains the assembly.  The supplied context provides box/cylinder/sphere/group
factories and a ``mats`` material dictionary; no external asset is used here.
Furniture fronts face local -Y unless explicitly documented otherwise.
"""

from math import pi, sin, cos
import random


def _root(C, name, parent, x, y, z, rot=0):
    ob = C.group(name, parent)
    ob.location = (x, y, z)
    ob.rotation_euler[2] = rot
    return ob


def _b(C, p, name, loc, size, mat, bevel=0.015):
    return C.box(name, loc, size, C.mats[mat], p, bevel=bevel)


def _c(C, p, name, loc, r, depth, mat, rotation=(0, 0, 0)):
    return C.cylinder(name, loc, r, depth, C.mats[mat], p, rotation=rotation)


def _s(C, p, name, loc, scale, mat, rotation=(0, 0, 0)):
    ob = C.sphere(name, loc, scale, C.mats[mat], p)
    ob.rotation_euler = rotation
    return ob


def _four_legs(C, p, half_x, half_y, height, mat="oak", radius=0.028):
    for i, (xx, yy) in enumerate(((-half_x, -half_y), (half_x, -half_y),
                                  (-half_x, half_y), (half_x, half_y))):
        _c(C, p, "Leg_%02d" % i, (xx, yy, height / 2), radius, height, mat)


def desk(C, parent, x, y, z, rot=0):
    """1.55 x .76 m workstation, monitor, keyboard, drawer and task light."""
    p = _root(C, "Workstation", parent, x, y, z, rot)
    _b(C, p, "Oak_worktop", (0, 0, .735), (1.55, .76, .05), "oak", .018)
    for xx in (-.68, .68):
        _b(C, p, "Steel_trestle", (xx, .02, .37), (.045, .62, .71), "white", .012)
    _b(C, p, "Rear_crossbar", (0, .30, .61), (1.38, .035, .06), "white")
    _b(C, p, "Cable_tray", (0, .22, .66), (.66, .15, .045), "black")
    _b(C, p, "Drawer_body", (.45, .035, .49), (.35, .57, .43), "white")
    for zz in (.36, .49, .62):
        _b(C, p, "Drawer_front", (.45, -.264, zz), (.33, .02, .117), "white", .005)
        _b(C, p, "Drawer_pull", (.45, -.29, zz + .025), (.14, .02, .014), "steel", .006)
    _b(C, p, "Monitor_foot", (0, .17, .773), (.23, .16, .016), "black", .008)
    _b(C, p, "Monitor_stand", (0, .21, .91), (.05, .036, .27), "steel", .009)
    _b(C, p, "Monitor_bezel", (0, .20, 1.12), (.60, .035, .35), "black", .014)
    _b(C, p, "Monitor_display", (0, .179, 1.124), (.57, .004, .316), "screen", .007)
    _b(C, p, "Monitor_lower_chin", (0, .176, .96), (.54, .007, .012), "black", .002)
    _b(C, p, "Keyboard_base", (-.025, -.18, .773), (.44, .145, .024), "black", .008)
    for row in range(4):
        for col in range(13):
            _b(C, p, "Keyboard_key", (-.214 + col * .031, -.224 + row * .029, .788),
               (.026, .021, .009), "steel", .002)
    _b(C, p, "Keyboard_spacebar", (-.025, -.244, .79), (.17, .014, .007), "steel", .002)
    _b(C, p, "Mouse_mat", (.32, -.20, .763), (.23, .20, .004), "leather", .012)
    _s(C, p, "Mouse", (.32, -.19, .78), (.03, .052, .019), "black")
    _c(C, p, "Lamp_base", (-.59, .18, .772), .073, .018, "brass")
    _c(C, p, "Lamp_stem", (-.59, .18, .945), .009, .33, "brass")
    _b(C, p, "Lamp_arm", (-.49, .18, 1.105), (.23, .02, .021), "brass", .008)
    _b(C, p, "Task_lamp_shade", (-.38, .18, 1.09), (.20, .09, .043), "black", .014)
    _b(C, p, "Task_lamp_diffuser", (-.38, .18, 1.066), (.16, .058, .003), "light", .001)
    _c(C, p, "Coffee_cup", (.60, -.17, .813), .036, .092, "ceramic")
    _c(C, p, "Coffee_surface", (.60, -.17, .86), .030, .0015, "walnut")
    _b(C, p, "Notebook", (-.61, -.19, .771), (.19, .25, .017), "paper", .002)
    _c(C, p, "Pen", (-.50, -.19, .784), .004, .15, "brass", (pi / 2, 0, .14))
    return p


def office_chair(C, parent, x, y, z, rot=0):
    """Ergonomic swivel chair with five rolling feet and separate upholstery."""
    p = _root(C, "Office_chair", parent, x, y, z, rot)
    _c(C, p, "Gas_lift", (0, 0, .28), .035, .32, "chrome")
    _c(C, p, "Lift_sleeve", (0, 0, .19), .049, .20, "black")
    for i in range(5):
        angle = i * 2 * pi / 5
        foot = _b(C, p, "Star_base_spoke", (.16 * cos(angle), .16 * sin(angle), .115),
                  (.32, .038, .025), "steel", .012)
        foot.rotation_euler[2] = angle
        _c(C, p, "Castor", (.31 * cos(angle), .31 * sin(angle), .047), .04, .047,
           "black", (pi / 2, 0, angle))
    _b(C, p, "Seat_shell", (0, -.012, .445), (.51, .49, .055), "black", .045)
    _b(C, p, "Seat_upholstery", (0, -.025, .492), (.49, .48, .075), "fabric_blue", .035)
    _b(C, p, "Seat_front_piping", (0, -.263, .49), (.42, .008, .008), "fabric", .004)
    _b(C, p, "Back_support", (0, .235, .64), (.09, .055, .37), "steel", .018)
    back = _b(C, p, "Back_shell", (0, .257, .85), (.47, .060, .55), "black", .055)
    back.rotation_euler[0] = -.08
    pad = _b(C, p, "Back_upholstery", (0, .215, .856), (.425, .053, .49), "fabric_blue", .055)
    pad.rotation_euler[0] = -.08
    _b(C, p, "Lumbar_seam", (0, .179, .76), (.36, .008, .009), "fabric", .003)
    for xx in (-.28, .28):
        _b(C, p, "Arm_support", (xx, .04, .59), (.024, .032, .23), "steel", .01)
        _b(C, p, "Arm_pad", (xx, -.02, .705), (.065, .30, .038), "black", .018)
    return p


def sofa(C, parent, x, y, z, rot=0):
    """Three-seat residential sofa, 2.35 x .91 m, with cushions and piping."""
    p = _root(C, "Three_seat_sofa", parent, x, y, z, rot)
    _four_legs(C, p, 1.00, .32, .15, "walnut", .034)
    _b(C, p, "Sofa_base", (0, 0, .27), (2.28, .84, .26), "fabric_cream", .065)
    _b(C, p, "Sofa_back", (0, .36, .64), (2.28, .19, .62), "fabric_cream", .065)
    for xx in (-1.06, 1.06):
        _b(C, p, "Sofa_arm", (xx, -.005, .48), (.20, .88, .50), "fabric_cream", .065)
    for xx in (-.655, 0, .655):
        _b(C, p, "Seat_cushion", (xx, -.055, .445), (.635, .66, .17), "fabric_cream", .065)
        _b(C, p, "Seat_cushion_piping", (xx, -.384, .45), (.545, .010, .010), "fabric", .004)
        pad = _b(C, p, "Back_cushion", (xx, .21, .716), (.63, .185, .42), "fabric_cream", .055)
        pad.rotation_euler[0] = -.12
    for xx, rz, mat in ((-.81, -.22, "fabric_blue"), (.81, .22, "fabric")):
        pillow = _b(C, p, "Throw_pillow", (xx, .005, .65), (.35, .16, .35), mat, .065)
        pillow.rotation_euler = (.18, rz, 0)
    return p


def armchair(C, parent, x, y, z, rot=0):
    p = _root(C, "Lounge_armchair", parent, x, y, z, rot)
    _four_legs(C, p, .32, .29, .21, "walnut", .030)
    _b(C, p, "Chair_base", (0, 0, .31), (.77, .76, .22), "fabric", .055)
    _b(C, p, "Chair_seat", (0, -.04, .445), (.59, .59, .13), "fabric_cream", .06)
    _b(C, p, "Chair_back", (0, .30, .67), (.74, .17, .64), "fabric", .06)
    pad = _b(C, p, "Chair_back_pad", (0, .18, .72), (.58, .15, .43), "fabric_cream", .065)
    pad.rotation_euler[0] = -.13
    for xx in (-.345, .345):
        _b(C, p, "Chair_arm", (xx, -.01, .56), (.11, .76, .18), "fabric", .045)
        _b(C, p, "Arm_oak_cap", (xx, -.01, .658), (.12, .64, .026), "walnut", .013)
    return p


def coffee_table(C, parent, x, y, z, rot=0):
    p = _root(C, "Coffee_table", parent, x, y, z, rot)
    _four_legs(C, p, .47, .24, .36, "walnut", .030)
    _b(C, p, "Table_top", (0, 0, .387), (1.15, .66, .055), "walnut", .035)
    _b(C, p, "Table_lower_shelf", (0, 0, .18), (1.02, .50, .022), "oak", .014)
    _b(C, p, "Art_book_bottom", (.16, .03, .426), (.30, .24, .024), "paper", .003)
    cover = _b(C, p, "Art_book_cover", (.16, .03, .440), (.302, .242, .006), "fabric_blue", .001)
    cover.rotation_euler[2] = .09
    _c(C, p, "Ceramic_bowl", (-.28, -.04, .451), .085, .075, "ceramic")
    _c(C, p, "Bowl_inset", (-.28, -.04, .49), .070, .002, "counter")
    return p


def _dining_chair(C, parent, x, y, rot):
    p = _root(C, "Dining_chair", parent, x, y, 0, rot)
    _four_legs(C, p, .19, .185, .43, "oak", .022)
    _b(C, p, "Dining_seat_frame", (0, 0, .44), (.47, .46, .04), "oak", .015)
    _b(C, p, "Dining_seat_cushion", (0, -.018, .48), (.43, .42, .055), "fabric_cream", .03)
    for xx in (-.204, .204):
        _b(C, p, "Dining_back_post", (xx, .197, .66), (.032, .034, .46), "oak", .014)
    _b(C, p, "Dining_back", (0, .20, .80), (.45, .054, .19), "oak", .028)
    _b(C, p, "Dining_back_pad", (0, .167, .797), (.38, .021, .13), "fabric_cream", .020)
    return p


def dining_set(C, parent, x, y, z, rot=0):
    """Six settings; allow a 3.0 x 2.3 m footprint plus circulation."""
    p = _root(C, "Six_person_dining_set", parent, x, y, z, rot)
    _four_legs(C, p, .77, .34, .715, "oak", .039)
    _b(C, p, "Dining_oak_top", (0, 0, .745), (1.95, .95, .06), "oak", .033)
    for xx in (-.56, .56):
        _dining_chair(C, p, xx, -.79, pi)
        _dining_chair(C, p, xx, .79, 0)
    _dining_chair(C, p, -1.22, 0, pi / 2)
    _dining_chair(C, p, 1.22, 0, -pi / 2)
    for xx in (-.54, .54):
        for yy in (-.30, .30):
            _c(C, p, "Dinner_plate", (xx, yy, .784), .115, .012, "ceramic")
            _c(C, p, "Plate_well", (xx, yy, .791), .085, .004, "white")
            _c(C, p, "Drinking_glass", (xx + .19, yy * .61, .835), .029, .115, "glass")
            _b(C, p, "Linen_napkin", (xx - .16, yy, .78), (.075, .15, .005), "fabric_cream", .002)
            _c(C, p, "Cutlery", (xx + .14, yy, .784), .004, .15, "chrome", (pi / 2, 0, 0))
    _c(C, p, "Center_vase", (0, 0, .86), .055, .17, "ceramic")
    for i in range(5):
        angle = i * 2 * pi / 5
        _c(C, p, "Flower_stem", (.03 * cos(angle), .03 * sin(angle), 1.025), .002, .30, "foliage")
        _s(C, p, "Flower_head", (.03 * cos(angle), .03 * sin(angle), 1.17 + .018 * (i % 2)),
           (.034, .034, .025), "white")
    return p


def kitchen(C, parent, x, y, z, rot=0):
    """4.75 m kitchen run with fitted oven, sink, hob and refrigerator.

    Cabinet backs are at local Y=.55. Keep that plane against the kitchen wall.
    The faucet and sink basin are assembled above a shallow inset counter well.
    """
    p = _root(C, "Fitted_kitchen", parent, x, y, z, rot)
    _b(C, p, "Recessed_plinth", (0, .24, .07), (3.60, .49, .14), "black", .003)
    _b(C, p, "Base_cabinet_bottom", (0, .235, .155), (3.60, .57, .03), "oak", .006)
    _b(C, p, "Base_cabinet_back", (0, .511, .49), (3.60, .018, .70), "oak", .004)
    for xx in (-1.785, -.9, -.15, 1.0, 1.785):
        _b(C, p, "Base_cabinet_partition", (xx, .235, .49), (.025, .57, .70), "oak", .004)
    for i, xx in enumerate((-1.35, -.45, .45, 1.35)):
        if i == 1:
            _b(C, p, "Oven_front", (xx, -.064, .485), (.82, .027, .64), "steel", .012)
            _b(C, p, "Oven_glass", (xx, -.081, .445), (.70, .009, .43), "black", .014)
            _b(C, p, "Oven_inner_glass", (xx, -.087, .445), (.58, .003, .32), "glass", .013)
            _b(C, p, "Oven_handle", (xx, -.125, .692), (.64, .028, .027), "chrome", .012)
            for dx in (-.25, .25):
                _c(C, p, "Oven_dial", (xx + dx, -.09, .764), .024, .018, "black", (pi / 2, 0, 0))
            _b(C, p, "Oven_clock", (xx, -.08, .765), (.17, .008, .038), "screen", .004)
        else:
            for dx in (-.22, .22):
                _b(C, p, "Oak_cabinet_door", (xx + dx, -.064, .49), (.43, .027, .69), "oak", .006)
                _b(C, p, "Cabinet_pull", (xx + dx, -.087, .76), (.18, .022, .014), "brass", .005)
    # The sink well is an actual opening between counter pieces, not a basin
    # laid over a solid slab.  The remaining worktop has a continuous front lip.
    _b(C, p, "Counter_left", (-.975, .235, .87), (1.65, .66, .06), "counter", .012)
    _b(C, p, "Counter_right", (1.40, .235, .87), (.80, .66, .06), "counter", .012)
    _b(C, p, "Counter_front_sink", (.425, -.04, .87), (1.15, .11, .06), "counter", .012)
    _b(C, p, "Counter_back_sink", (.425, .49, .87), (1.15, .15, .06), "counter", .012)
    _b(C, p, "Sink_bottom", (.425, .205, .766), (1.12, .37, .022), "steel", .008)
    for xx in (-.139, .989):
        _b(C, p, "Sink_side_wall", (xx, .205, .819), (.020, .41, .12), "steel", .006)
    for yy in (.013, .407):
        _b(C, p, "Sink_end_wall", (.425, yy, .819), (1.13, .018, .12), "steel", .006)
    _c(C, p, "Sink_drain", (.425, .205, .780), .027, .003, "chrome")
    _c(C, p, "Faucet_riser", (.425, .47, 1.02), .021, .30, "chrome")
    _c(C, p, "Faucet_spout", (.425, .38, 1.17), .021, .19, "chrome", (pi / 2, 0, 0))
    _c(C, p, "Faucet_tip", (.425, .285, 1.14), .021, .062, "chrome")
    _c(C, p, "Tap_lever", (.50, .47, .99), .011, .13, "chrome", (0, pi / 2, 0))
    _b(C, p, "Induction_hob", (-.55, .22, .906), (.72, .48, .010), "black", .009)
    for xx in (-.75, -.36):
        for yy in (.08, .36):
            _c(C, p, "Hob_ring", (xx, yy, .912), .084, .0015, "steel")
            _c(C, p, "Hob_ring_center", (xx, yy, .913), .076, .001, "black")
    _b(C, p, "Backsplash", (0, .569, 1.21), (3.60, .025, .65), "tile", .004)
    for xx in (-1.36, .45, 1.35):
        _b(C, p, "Upper_cabinet", (xx, .40, 1.79), (.87, .34, .74), "white", .010)
        for dx in (-.215, .215):
            _b(C, p, "Upper_door", (xx + dx, .22, 1.79), (.421, .022, .722), "white", .007)
            _b(C, p, "Upper_pull", (xx + dx, .203, 1.48), (.14, .019, .012), "brass", .004)
        _b(C, p, "Undercabinet_strip", (xx, .25, 1.413), (.78, .03, .008), "light", .002)
    _b(C, p, "Extractor_hood", (-.45, .38, 1.57), (.86, .43, .10), "steel", .022)
    _b(C, p, "Extractor_flue", (-.45, .46, 1.93), (.31, .21, .65), "steel", .018)
    _b(C, p, "Fridge_body", (2.26, .16, 1.045), (.82, .81, 2.09), "steel", .030)
    _b(C, p, "Fridge_door", (2.26, -.262, 1.355), (.79, .039, 1.405), "steel", .018)
    _b(C, p, "Freezer_drawer", (2.26, -.262, .337), (.79, .039, .58), "steel", .018)
    _b(C, p, "Fridge_handle", (1.967, -.305, 1.27), (.026, .032, .60), "chrome", .012)
    _b(C, p, "Freezer_handle", (2.26, -.307, .57), (.57, .032, .025), "chrome", .010)
    _b(C, p, "Fridge_display", (2.12, -.284, 1.63), (.14, .005, .075), "screen", .004)
    _b(C, p, "Cutting_board", (1.41, .17, .914), (.44, .29, .02), "walnut", .015)
    _c(C, p, "Utensil_crock", (-1.49, .37, 1.00), .061, .20, "ceramic")
    for i in range(3):
        _c(C, p, "Wooden_utensil", (-1.53 + .04 * i, .37, 1.145), .009, .25, "oak", (0, .10 * (i - 1), 0))
        _s(C, p, "Spoon_head", (-1.54 + .05 * i, .37, 1.285), (.023, .01, .04), "oak")
    return p


def bed(C, parent, x, y, z, rot=0):
    """King bed, 1.90 x 2.18 m; headboard lies toward local +Y."""
    p = _root(C, "King_bed", parent, x, y, z, rot)
    _four_legs(C, p, .78, .90, .16, "walnut", .037)
    _b(C, p, "Bed_frame", (0, 0, .285), (1.95, 2.15, .28), "walnut", .025)
    _b(C, p, "Bed_mattress", (0, -.025, .492), (1.87, 2.04, .25), "white", .070)
    _b(C, p, "Mattress_edge_seam", (0, -1.039, .48), (1.72, .010, .012), "fabric_cream", .004)
    _b(C, p, "Upholstered_headboard", (0, 1.085, .73), (2.03, .16, 1.31), "fabric", .07)
    for xx in (-.65, 0, .65):
        _b(C, p, "Headboard_panel", (xx, .993, .85), (.63, .035, .83), "fabric_cream", .045)
    _b(C, p, "Duvet", (0, -.32, .655), (1.89, 1.42, .15), "fabric_cream", .073)
    _b(C, p, "Duvet_fold", (0, .35, .72), (1.82, .21, .068), "white", .028)
    _b(C, p, "Bed_runner", (0, -.64, .738), (1.90, .43, .028), "fabric_blue", .012)
    for xx, rr in ((-.47, -.06), (.47, .06)):
        pillow = _b(C, p, "Sleeping_pillow", (xx, .71, .692), (.77, .42, .16), "white", .075)
        pillow.rotation_euler[2] = rr
        small = _b(C, p, "Decorative_pillow", (xx, .48, .771), (.40, .18, .30), "fabric", .070)
        small.rotation_euler[0] = -.24
    for xx in (-1.34, 1.34):
        _b(C, p, "Nightstand", (xx, .75, .32), (.52, .48, .57), "walnut", .018)
        _b(C, p, "Nightstand_drawer", (xx, .502, .42), (.47, .015, .22), "oak", .008)
        _b(C, p, "Nightstand_pull", (xx, .483, .45), (.12, .018, .012), "brass", .005)
        _c(C, p, "Bedside_lamp_base", (xx, .76, .621), .09, .021, "brass")
        _c(C, p, "Bedside_lamp_stem", (xx, .76, .785), .012, .31, "brass")
        _c(C, p, "Bedside_lamp_shade", (xx, .76, .94), .14, .22, "fabric_cream")
        _c(C, p, "Bedside_diffuser", (xx, .76, .827), .115, .006, "light")
    return p


def plant(C, parent, x, y, z, rot=0):
    """1.45 m broadleaf houseplant with individual naturally arranged leaves."""
    p = _root(C, "Indoor_plant", parent, x, y, z, rot)
    _c(C, p, "Planter_foot", (0, 0, .018), .175, .035, "ceramic")
    _c(C, p, "Planter", (0, 0, .225), .205, .42, "ceramic")
    _c(C, p, "Planter_rim", (0, 0, .433), .216, .028, "ceramic")
    _c(C, p, "Potting_soil", (0, 0, .440), .189, .013, "soil")
    for i in range(5):
        angle = i * 2.39996
        stem_x, stem_y = .070 * cos(angle), .070 * sin(angle)
        top = 1.22 + .15 * sin(i * 1.9)
        _c(C, p, "Plant_stem", (stem_x, stem_y, (.445 + top) / 2), .011,
           top - .445, "walnut")
        for j in range(3):
            a = angle + j * 2.1
            height = top - j * .22
            radius = .12 + j * .023
            _s(C, p, "Broad_leaf", (stem_x + radius * cos(a), stem_y + radius * sin(a), height),
               (.075, .21, .018), "foliage", (.19 + .12 * j, .12, a - pi / 2))
    return p


def bookshelf(C, parent, x, y, z, rot=0):
    """1.15 m wide open shelving with deterministic book and object dressing."""
    p = _root(C, "Bookshelf", parent, x, y, z, rot)
    _b(C, p, "Shelf_back", (0, .16, 1.09), (1.15, .025, 2.18), "oak", .004)
    for xx in (-.555, .555):
        _b(C, p, "Shelf_side", (xx, -.035, 1.09), (.04, .42, 2.18), "oak", .008)
    for zz in (.08, .51, .94, 1.37, 1.80, 2.16):
        _b(C, p, "Shelf_board", (0, -.035, zz), (1.15, .42, .038), "oak", .008)
    rng = random.Random(177)
    colors = ("fabric_blue", "fabric", "paper", "walnut", "white")
    for shelf in range(5):
        base = (.08, .51, .94, 1.37, 1.80)[shelf] + .019
        cursor = -.47
        for i in range(7 if shelf != 2 else 4):
            width = rng.uniform(.033, .071)
            height = rng.uniform(.20, .30)
            _b(C, p, "Book_pages", (cursor + width / 2, -.066, base + height / 2),
               (width - .008, .22, height - .009), "paper", .001)
            _b(C, p, "Book_spine", (cursor + width / 2, -.184, base + height / 2),
               (width, .014, height), colors[(shelf + i) % len(colors)], .002)
            for dz in (.05, height - .05):
                _b(C, p, "Book_spine_rule", (cursor + width / 2, -.192, base + dz),
                   (width * .65, .002, .004), "brass", .0005)
            cursor += width + .009
        _c(C, p, "Shelf_ceramic", (.33, -.035, base + .09), .067, .18, "ceramic")
    return p


def bathroom(C, parent, x, y, z, rot=0):
    """Vanity and toilet; footprint 2.25 x 1.45 m, rear wall at Y=.45."""
    p = _root(C, "Bathroom_fixtures", parent, x, y, z, rot)
    _b(C, p, "Vanity_cabinet", (-.55, .075, .435), (.98, .56, .72), "oak", .015)
    for xx in (-.80, -.30):
        _b(C, p, "Vanity_door", (xx, -.218, .44), (.477, .025, .68), "oak", .007)
        _b(C, p, "Vanity_pull", (xx, -.241, .69), (.15, .018, .012), "brass", .004)
    _b(C, p, "Vanity_stone_top", (-.55, .065, .823), (1.03, .61, .056), "counter", .014)
    _s(C, p, "Washbasin", (-.55, -.02, .891), (.33, .215, .087), "ceramic")
    _s(C, p, "Basin_inner", (-.55, -.045, .932), (.275, .167, .014), "white")
    _c(C, p, "Basin_drain", (-.55, -.05, .946), .022, .002, "chrome")
    _c(C, p, "Vanity_faucet", (-.55, .264, .991), .017, .29, "chrome")
    _c(C, p, "Vanity_spout", (-.55, .19, 1.132), .017, .16, "chrome", (pi / 2, 0, 0))
    _b(C, p, "Mirror_frame", (-.55, .393, 1.55), (.98, .04, 1.02), "brass", .028)
    _b(C, p, "Mirror_glass", (-.55, .367, 1.55), (.93, .007, .97), "chrome", .017)
    _b(C, p, "Vanity_light", (-.55, .32, 2.12), (.72, .08, .045), "light", .020)
    _c(C, p, "Soap_dispenser", (-.91, .17, .951), .035, .20, "ceramic")
    _b(C, p, "Soap_pump", (-.91, .155, 1.06), (.06, .05, .015), "chrome", .006)
    _s(C, p, "Toilet_pedestal", (.66, -.02, .215), (.24, .31, .215), "ceramic")
    _s(C, p, "Toilet_bowl", (.66, -.12, .365), (.29, .37, .16), "ceramic")
    _s(C, p, "Toilet_seat", (.66, -.155, .484), (.277, .343, .026), "white")
    _s(C, p, "Toilet_seat_inset", (.66, -.155, .502), (.196, .25, .012), "counter")
    _b(C, p, "Toilet_cistern", (.66, .243, .64), (.44, .21, .49), "ceramic", .045)
    _b(C, p, "Toilet_cistern_lid", (.66, .243, .89), (.46, .23, .035), "ceramic", .019)
    _c(C, p, "Dual_flush_button", (.66, .235, .91), .026, .005, "chrome")
    _b(C, p, "Towel_rail", (-1.115, .015, .86), (.023, .36, .023), "chrome", .009)
    _b(C, p, "Hanging_towel", (-1.134, .015, .63), (.013, .30, .46), "fabric_cream", .012)
    return p


def meeting_table(C, parent, x, y, z, rot=0):
    """Eight-person meeting table; assembly footprint approx. 3.9 x 2.6 m."""
    p = _root(C, "Meeting_table", parent, x, y, z, rot)
    _b(C, p, "Conference_top", (0, 0, .746), (2.85, 1.14, .058), "walnut", .06)
    for xx in (-.90, .90):
        _b(C, p, "Conference_pedestal", (xx, 0, .37), (.11, .69, .71), "steel", .021)
        _b(C, p, "Conference_foot", (xx, 0, .033), (.68, .78, .046), "steel", .020)
    for xx in (-.89, 0, .89):
        office_chair(C, p, xx, -.95, 0, pi)
        office_chair(C, p, xx, .95, 0, 0)
    office_chair(C, p, -1.79, 0, 0, pi / 2)
    office_chair(C, p, 1.79, 0, 0, -pi / 2)
    _b(C, p, "Cable_grommet", (0, 0, .778), (.32, .14, .006), "steel", .008)
    _b(C, p, "Conference_speaker", (0, .015, .816), (.25, .16, .07), "black", .035)
    for xx in (-.86, .86):
        _b(C, p, "Meeting_notepad", (xx, -.28, .785), (.22, .28, .018), "paper", .003)
        _c(C, p, "Meeting_water_glass", (xx + .22, -.27, .835), .03, .116, "glass")
    return p


def pendant(C, parent, x, y, z, rot=0):
    """Ceiling fixture: z marks ceiling level; fixture hangs .64 m below it."""
    p = _root(C, "Pendant_light", parent, x, y, z, rot)
    _c(C, p, "Ceiling_canopy", (0, 0, -.018), .10, .036, "brass")
    _c(C, p, "Pendant_cord", (0, 0, -.28), .004, .52, "black")
    _c(C, p, "Pendant_shade", (0, 0, -.58), .24, .20, "brass")
    _c(C, p, "Pendant_diffuser", (0, 0, -.683), .215, .006, "light")
    return p


__all__ = ["desk", "office_chair", "sofa", "armchair", "coffee_table",
           "dining_set", "kitchen", "bed", "plant", "bookshelf", "bathroom",
           "meeting_table", "pendant"]
