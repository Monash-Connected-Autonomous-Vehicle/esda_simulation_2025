#!/usr/bin/env python3
"""Flatten description/robot.urdf.xacro into a URDF that Isaac Sim can import.

WHY THIS EXISTS
    Isaac Sim's URDF importer wants one self-contained URDF with resolvable mesh
    paths.  What the repo has instead is a xacro tree with "package://" URIs, ten
    includes, conditional ros2_control blocks, and ten <gazebo> extensions that
    mean nothing outside gz-sim.

    It also carries mass values that would wreck a PhysX simulation -- see
    research/isaac/config/robot_mass.yaml for the full argument.  Fixing them in
    the xacro is not an option: Gazebo, the real robot's ros2_control chain and
    the recorded maps all depend on the current description, and
    diff_drive_controller's open_loop mode makes the bad numbers harmless there.
    So the correction lives here, in the export, where only Isaac sees it.

NO ROS REQUIRED
    xacro is on PyPI and every xacro:include in this repo uses a *relative*
    filename.  "$(find esda_simulation_2025)" appears only inside
    description/ros2_control_2wd.xacro, which use_ros2_control:=false excludes.
    So this runs under plain CPython on Windows -- verified against Isaac Sim's
    own Python 3.12 -- and also under ROS Humble's 3.10 in WSL.

WHAT IT DOES, IN ORDER
    1. xacro -> DOM, with use_ros2_control:=false by default.
    2. Strip comments.  Not cosmetic: robot_visual.xacro and camera.xacro park
       dead <mesh> refs inside comments, including one pointing at a package
       ("esda_bot") that does not exist in this repo.  Left in place they look
       like missing assets.
    3. Strip <gazebo> elements -- sensors, plugins and friction.  Isaac gets its
       own sensors; the friction is reapplied from robot_mass.yaml by
       research/isaac/scripts/convert_robot.py.
    4. Drop links named by --drop (default: appearance_body).  That link carries
       ~30 STL visuals with no collision and no inertial.  Harmless in Gazebo,
       pure render cost once cloned across thousands of envs.
    5. Rewrite "package://esda_simulation_2025/assets/X.stl" to "meshes/X.stl"
       and copy only the files still referenced.
    6. Apply the mass/inertia overrides.
    7. Report the resulting mass budget and centre of mass, because that is the
       number that decides whether the robot survives the 8.5 degree ramp.

USAGE
    python3 tools/export_urdf_for_isaac.py
    python3 tools/export_urdf_for_isaac.py --appearance \
        --out research/isaac/assets/esda_robot_visual.urdf
    python3 tools/export_urdf_for_isaac.py --no-mass-override
"""
import argparse
import math
import os
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, "src", "esda_simulation_2025")
DEFAULT_XACRO = os.path.join(PKG, "description", "robot.urdf.xacro")
DEFAULT_OUT = os.path.join(REPO, "research", "isaac", "assets", "esda_robot.urdf")
DEFAULT_MASS = os.path.join(REPO, "research", "isaac", "config", "robot_mass.yaml")

PKG_PREFIX = "package://esda_simulation_2025/"
MESH_SUBDIR = "meshes"
INERTIA_KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")

# base_link rides the wheel axle, so CoM height above ground needs this added.
WHEEL_RADIUS = 0.1625
# The three links that actually touch the ground, for the load-split report.
SUPPORT_LINKS = ("left_wheel", "right_wheel", "caster_wheel")


# ------------------------------------------------------------------ xacro --
def run_xacro(path, mappings):
    try:
        import xacro
    except ImportError:
        sys.exit(
            "xacro is not importable.\n"
            "  Windows / Isaac:  C:\\isaac-sim\\python.bat -m pip install xacro\n"
            "  WSL / ROS:        already provided by ros-humble-xacro"
        )
    return xacro.process_file(path, mappings=mappings)


# -------------------------------------------------------------------- DOM --
def strip_comments(node):
    """Remove every comment node, recursively. Returns how many went."""
    gone = 0
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
            gone += 1
        elif child.nodeType == child.ELEMENT_NODE:
            gone += strip_comments(child)
    return gone


def strip_tag(root, tag):
    gone = 0
    for el in list(root.getElementsByTagName(tag)):
        if el.parentNode is not None:
            el.parentNode.removeChild(el)
            gone += 1
    return gone


def drop_links(root, names):
    """Remove the named links and every joint that touches them."""
    dropped_links, dropped_joints = [], []
    for link in list(root.getElementsByTagName("link")):
        if link.getAttribute("name") in names:
            link.parentNode.removeChild(link)
            dropped_links.append(link.getAttribute("name"))
    for joint in list(root.getElementsByTagName("joint")):
        kids = [c.getAttribute("link") for c in joint.getElementsByTagName("child")]
        parents = [c.getAttribute("link") for c in joint.getElementsByTagName("parent")]
        if names & set(kids + parents):
            joint.parentNode.removeChild(joint)
            dropped_joints.append(joint.getAttribute("name"))
    return dropped_links, dropped_joints


def rewrite_meshes(root):
    """package:// -> relative, and report what needs copying.

    Returns ({source_abs_path: basename}, [unresolved URIs]).  A package:// URI
    for some other package is reported rather than silently rewritten into
    something that will not resolve.
    """
    wanted, foreign = {}, []
    for mesh in root.getElementsByTagName("mesh"):
        fn = mesh.getAttribute("filename")
        if not fn.startswith("package://"):
            continue
        if not fn.startswith(PKG_PREFIX):
            foreign.append(fn)
            continue
        rel = fn[len(PKG_PREFIX):]
        base = os.path.basename(rel)
        wanted[os.path.join(PKG, rel.replace("/", os.sep))] = base
        mesh.setAttribute("filename", MESH_SUBDIR + "/" + base)
    return wanted, foreign


# ------------------------------------------------------------------- mass --
def apply_mass_overrides(root, cfg):
    """Scale <mass>/<inertia> per robot_mass.yaml.

    Inertia is scaled by the mass ratio rather than recomputed: for fixed
    geometry it is linear in mass, and the xacro's inertial_box /
    inertial_cylinder / inertial_sphere macros already encode the correct shape.
    Only the scale was wrong.  That keeps geometry assumptions out of the config.
    """
    links_cfg = (cfg or {}).get("links") or {}
    applied, massless = [], []
    unknown = sorted(links_cfg)

    for link in root.getElementsByTagName("link"):
        name = link.getAttribute("name")
        inertial_els = link.getElementsByTagName("inertial")
        if not inertial_els:
            massless.append(name)
            continue
        inertial = inertial_els[0]
        mass_el = inertial.getElementsByTagName("mass")[0]
        old = float(mass_el.getAttribute("value"))

        spec = links_cfg.get(name)
        if spec is None:
            applied.append((name, old, old))
            continue
        if name in unknown:
            unknown.remove(name)

        new = float(spec["mass"])
        mass_el.setAttribute("value", repr(new))

        ratio = (new / old) if old else 0.0
        for inert in inertial.getElementsByTagName("inertia"):
            for key in INERTIA_KEYS:
                if inert.hasAttribute(key):
                    scaled = float(inert.getAttribute(key)) * ratio
                    inert.setAttribute(key, repr(scaled))

        com = spec.get("com_xyz")
        if com is not None:
            origins = inertial.getElementsByTagName("origin")
            if origins:
                origin = origins[0]
            else:
                origin = inertial.ownerDocument.createElement("origin")
                inertial.appendChild(origin)
            origin.setAttribute("xyz", " ".join(repr(float(v)) for v in com))

        applied.append((name, old, new))

    return applied, massless, unknown


# -------------------------------------------------- mass budget reporting --
def _xyz(el, attr, default=(0.0, 0.0, 0.0)):
    if el is None or not el.hasAttribute(attr):
        return default
    return tuple(float(v) for v in el.getAttribute(attr).split())


def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _apply(mat, vec):
    return tuple(sum(mat[i][j] * vec[j] for j in range(3)) for i in range(3))


def _compose(parent_R, parent_t, child_R, child_t):
    rotated = _apply(parent_R, child_t)
    t = tuple(parent_t[i] + rotated[i] for i in range(3))
    R = tuple(
        tuple(sum(parent_R[i][k] * child_R[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )
    return R, t


def mass_budget(root):
    """Total mass and centre of mass in the root link's frame.

    Joint angles are taken as zero, which is exactly the spawn pose.  The point
    of this number is the CoM *height*: a tall, light robot tips on the 8.5
    degree ramp, and this is what says whether it will.
    """
    joints = []
    for j in root.getElementsByTagName("joint"):
        parent = j.getElementsByTagName("parent")
        child = j.getElementsByTagName("child")
        if not parent or not child:
            continue
        origins = j.getElementsByTagName("origin")
        origin = origins[0] if origins else None
        joints.append((
            parent[0].getAttribute("link"),
            child[0].getAttribute("link"),
            _xyz(origin, "xyz"),
            _xyz(origin, "rpy"),
        ))

    names = set(l.getAttribute("name") for l in root.getElementsByTagName("link"))
    children = set(c for _, c, _, _ in joints)
    roots = sorted(names - children)
    if not roots:
        return 0.0, (0.0, 0.0, 0.0), "<none>", [], {}
    base = roots[0]

    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    pose = {base: (identity, (0.0, 0.0, 0.0))}
    pending = list(joints)
    while pending:
        progressed = False
        for item in list(pending):
            parent, child, xyz, rpy = item
            if parent in pose and child not in pose:
                pR, pt = pose[parent]
                pose[child] = _compose(pR, pt, _rpy_matrix(*rpy), xyz)
                pending.remove(item)
                progressed = True
        if not progressed:
            break
    unreached = sorted(set(c for _, c, _, _ in pending))

    total = 0.0
    moment = [0.0, 0.0, 0.0]
    for link in root.getElementsByTagName("link"):
        inertial_els = link.getElementsByTagName("inertial")
        if not inertial_els:
            continue
        name = link.getAttribute("name")
        if name not in pose:
            continue
        mass = float(inertial_els[0].getElementsByTagName("mass")[0].getAttribute("value"))
        origins = inertial_els[0].getElementsByTagName("origin")
        local = _xyz(origins[0] if origins else None, "xyz")
        R, t = pose[name]
        rotated = _apply(R, local)
        total += mass
        for i in range(3):
            moment[i] += mass * (t[i] + rotated[i])

    com = tuple(m / total for m in moment) if total else (0.0, 0.0, 0.0)
    support = {name: pose[name][1][0] for name in SUPPORT_LINKS if name in pose}
    return total, com, base, unreached, support


def load_split(com_x, support):
    """Static fraction of weight on the drive axle vs the caster.

    This is the traction number.  The drive wheels sit at base_link x 0 and the
    caster trails at about -0.725, so a CoM behind the axle unloads the driven
    wheels onto a skid that robot_core_ref.xacro deliberately gave mu 0.001 --
    and the robot then cannot climb the 8.5 degree ramp in
    worlds/models/orange_igvc/model.sdf.  Gazebo hides this entirely because
    diff_drive_controller runs open_loop and never consults traction.
    """
    axle = support.get("left_wheel", support.get("right_wheel"))
    caster = support.get("caster_wheel")
    if axle is None or caster is None or abs(axle - caster) < 1e-9:
        return None
    frac_axle = (caster - com_x) / (caster - axle)
    return frac_axle, axle, caster


# ------------------------------------------------------------------- main --
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--xacro", default=DEFAULT_XACRO)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--mass-config", default=DEFAULT_MASS)
    ap.add_argument("--no-mass-override", action="store_true",
                    help="keep the xacro's own masses (the 1.25 kg robot)")
    ap.add_argument("--appearance", action="store_true",
                    help="keep appearance_body (demo asset, not training)")
    ap.add_argument("--drop", action="append", default=None,
                    help="extra link name to drop; repeatable")
    ap.add_argument("--mapping", action="append", default=None,
                    metavar="NAME:=VALUE", help="extra xacro argument; repeatable")
    args = ap.parse_args(argv)

    mappings = {"use_ros2_control": "false", "sim_mode": "true"}
    for item in args.mapping or []:
        if ":=" not in item:
            ap.error("--mapping needs NAME:=VALUE, got %r" % item)
        key, value = item.split(":=", 1)
        mappings[key] = value

    doc = run_xacro(args.xacro, mappings)
    root = doc.documentElement

    n_comments = strip_comments(root)
    n_gazebo = strip_tag(root, "gazebo")
    n_ros2 = strip_tag(root, "ros2_control")
    n_trans = strip_tag(root, "transmission")

    to_drop = set(args.drop or [])
    if not args.appearance:
        to_drop.add("appearance_body")
    if to_drop:
        dropped_links, dropped_joints = drop_links(root, to_drop)
    else:
        dropped_links, dropped_joints = [], []

    meshes, foreign = rewrite_meshes(root)

    cfg = None
    if not args.no_mass_override:
        try:
            import yaml
        except ImportError:
            sys.exit("pyyaml is required for --mass-config; "
                     "pass --no-mass-override to skip")
        with open(args.mass_config) as fh:
            cfg = yaml.safe_load(fh)
    applied, massless, unknown = apply_mass_overrides(root, cfg)

    out_path = os.path.abspath(args.out)
    out_dir = os.path.dirname(out_path)
    mesh_dir = os.path.join(out_dir, MESH_SUBDIR)
    os.makedirs(mesh_dir, exist_ok=True)
    copied, missing = [], []
    for src, base in sorted(meshes.items()):
        if not os.path.exists(src):
            missing.append(src)
            continue
        dst = os.path.join(mesh_dir, base)
        fresh = os.path.exists(dst) and os.path.getsize(dst) == os.path.getsize(src)
        if not fresh:
            shutil.copy2(src, dst)
        copied.append(base)

    xml = root.toprettyxml(indent="  ")
    xml = "\n".join(line for line in xml.splitlines() if line.strip())
    if xml.startswith("<?xml"):
        xml = "\n".join(xml.splitlines()[1:])
    mass_note = ("xacro originals (NOT physical)" if args.no_mass_override
                 else "research/isaac/config/robot_mass.yaml")
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write('<?xml version="1.0"?>\n')
        fh.write("<!-- GENERATED by tools/export_urdf_for_isaac.py. "
                 "Do not edit by hand.\n")
        fh.write("     Source: src/esda_simulation_2025/description/"
                 "robot.urdf.xacro\n")
        fh.write("     Masses: %s -->\n" % mass_note)
        fh.write(xml + "\n")

    total, com, base, unreached, support = mass_budget(root)
    n_links = len(root.getElementsByTagName("link"))
    n_joints = len(root.getElementsByTagName("joint"))

    print("wrote %s" % out_path)
    print("  stripped : %d comments, %d <gazebo>, %d <ros2_control>, "
          "%d <transmission>" % (n_comments, n_gazebo, n_ros2, n_trans))
    if dropped_links:
        print("  dropped  : links %s / joints %s"
              % (", ".join(dropped_links), ", ".join(dropped_joints)))
    print("  kept     : %d links, %d joints" % (n_links, n_joints))
    print("  meshes   : %d copied to %s/" % (len(copied), MESH_SUBDIR))
    for fn in foreign:
        print("  WARNING  : unresolved package:// URI left as-is: %s" % fn)
    for fn in missing:
        print("  WARNING  : referenced mesh not found on disk: %s" % fn)
    if unknown:
        print("  WARNING  : robot_mass.yaml names links not in the URDF: %s"
              % ", ".join(unknown))
    if unreached:
        print("  WARNING  : links not reachable from %s: %s"
              % (base, ", ".join(unreached)))
    print("  massless : %s" % (", ".join(massless) or "none"))
    print("  root link: %s" % base)
    print("  mass budget:")
    for name, old, new in sorted(applied, key=lambda r: -r[2]):
        flag = "" if abs(new - old) < 1e-12 else "   <- was %.3f" % old
        print("    %-14s %8.3f kg%s" % (name, new, flag))
    print("    %-14s %8.3f kg total" % ("", total))
    print("  centre of mass in %s: x %+.4f  y %+.4f  z %+.4f"
          % (base, com[0], com[1], com[2]))
    print("    CoM above ground = z + wheel radius %.4f = %.4f m"
          % (WHEEL_RADIUS, com[2] + WHEEL_RADIUS))

    split = load_split(com[0], support)
    if split is None:
        print("  static load split: not computable (support links missing)")
    else:
        frac_axle, axle_x, caster_x = split
        print("  static load split (drive axle x %+.3f, caster x %+.3f):"
              % (axle_x, caster_x))
        print("    drive wheels %5.1f%%  (%6.2f kg,  %5.2f kg per wheel)"
              % (100.0 * frac_axle, total * frac_axle, total * frac_axle / 2.0))
        print("    caster       %5.1f%%  (%6.2f kg)"
              % (100.0 * (1.0 - frac_axle), total * (1.0 - frac_axle)))
        if frac_axle < 0.5:
            print("    WARNING: most of the weight is on the caster, which"
                  " robot_core_ref.xacro gives mu 0.001.")
            print("             Traction will be poor and the 8.5 degree ramp"
                  " may be unclimbable.")
            print("             Move chassis com_xyz forward in"
                  " research/isaac/config/robot_mass.yaml.")
    print("  NOTE: these masses are estimates. Weigh the vehicle and replace"
          " them before trusting any trained policy.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
