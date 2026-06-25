"""
add_body_to_isaac_sim.py — Add ZEKO torso body to the existing Isaac Sim scene.

Run this from the Isaac Sim Script Editor ONCE after loading your scene.
It adds the body mesh as a visual-only prim under pelvis_mount_link without
touching any physics, joints, or the existing USD structure.

BEFORE RUNNING:
  1. Load your zeko.usda scene in Isaac Sim
  2. Update OBJ_PATH below to the absolute path of zeko_body.obj on your machine
  3. Open Script Editor (Window → Script Editor)
  4. Paste or load this script and click Run

WHAT IT DOES:
  - Creates a new Xform prim: /zeko/Geometry/base_link/support_frame_link/pelvis_mount_link/body_visual
  - References zeko_body.obj from your meshes folder
  - Applies the correct transform so the body sits exactly on top of pelvis_mount_link:
      scale    = 0.01 (OBJ is in cm, scene is in metres)
      rotateZ  = -90° (aligns OBJ Y-forward with robot X-forward)
      translate = (0, 0.28, 0.792) in pelvis_mount_link local frame
        0    = centred front-back (OBJ Y-axis is already centred)
        0.28 = centres body left-right (OBJ X-centre is at 28 cm, not 0)
        0.792 = lifts body so bottom (OBJ Z=-64 cm → -0.64 m) sits at
                top of pelvis_mount_link (z=0.152 m in pelvis local frame)
                  0.152 - (-0.64) = 0.792 m

  - Marks the prim as purpose="render" (visual only, no physics)
  - Does NOT add collision, mass, or joints — purely decorative

AFTER RUNNING:
  Check the Viewport — the body should appear sitting on the robot.
  If it looks offset, adjust TRANSLATE_X/Y/Z below and re-run.
  Then save the scene (Ctrl+S) to persist the change in your USD.
"""

import omni.usd
from pxr import UsdGeom, Gf, Sdf, UsdShade

# ── UPDATE THIS to the absolute path of zeko_body.obj on your machine ────────
OBJ_PATH = "/home/zeko-sim/Desktop/robot-base-simulation/meshes/zeko_body.obj"
# Example: "/home/zeko-sim/robot-base-simulation/meshes/zeko_body.obj"

# ── Prim paths (match your Stage panel — verified from your scene) ─────────────
PARENT_PATH = "/zeko/Geometry/base_link/support_frame_link/pelvis_mount_link"
BODY_PATH   = PARENT_PATH + "/body_visual"

# ── Transform values (calculated from OBJ geometry analysis) ──────────────────
# Adjust these if the body appears misaligned in the Viewport after running.
TRANSLATE_X  =  0.0     # m — forward/backward offset from pelvis centre
TRANSLATE_Y  =  0.28    # m — left/right offset (centres the OBJ X-axis at 0)
TRANSLATE_Z  =  0.792   # m — vertical: lifts OBJ bottom to pelvis_mount top
ROTATE_Z_DEG = -90.0    # deg — aligns OBJ Y (body front-back) with robot X (forward)
SCALE        =  0.01    # OBJ is in cm, scene is in metres

# ── Main ──────────────────────────────────────────────────────────────────────

def add_body():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[ZEKO Body] ERROR: No stage open. Load your scene first.")
        return

    # Verify parent exists
    parent_prim = stage.GetPrimAtPath(PARENT_PATH)
    if not parent_prim.IsValid():
        print(f"[ZEKO Body] ERROR: Parent prim not found: {PARENT_PATH}")
        print("[ZEKO Body] Check your Stage panel for the correct pelvis_mount_link path.")
        return

    # Remove existing body_visual if re-running
    existing = stage.GetPrimAtPath(BODY_PATH)
    if existing.IsValid():
        stage.RemovePrim(BODY_PATH)
        print("[ZEKO Body] Removed existing body_visual prim (re-running)")

    # Define new Xform prim
    body_xform = UsdGeom.Xform.Define(stage, BODY_PATH)
    body_prim  = stage.GetPrimAtPath(BODY_PATH)

    # Set purpose to render (visual only, not included in physics or collision)
    imageable = UsdGeom.Imageable(body_prim)
    imageable.CreatePurposeAttr().Set(UsdGeom.Tokens.render)

    # Apply transform: scale → rotateZ → translate (USD applies right-to-left)
    xformable = UsdGeom.Xformable(body_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(TRANSLATE_X, TRANSLATE_Y, TRANSLATE_Z))
    xformable.AddRotateZOp().Set(ROTATE_Z_DEG)
    xformable.AddScaleOp().Set(Gf.Vec3f(SCALE, SCALE, SCALE))

    # Reference the OBJ file
    # Isaac Sim 6.0 can reference OBJ files directly in USD as external assets.
    # The OBJ is loaded as a visual mesh; MTL file is picked up automatically
    # as long as zeko_body.mtl is in the same folder as zeko_body.obj.
    body_prim.GetReferences().AddReference(OBJ_PATH)

    print(f"[ZEKO Body] Body mesh added at: {BODY_PATH}")
    print(f"[ZEKO Body] Referenced: {OBJ_PATH}")
    print(f"[ZEKO Body] Transform: translate=({TRANSLATE_X}, {TRANSLATE_Y}, {TRANSLATE_Z})"
          f"  rotateZ={ROTATE_Z_DEG}°  scale={SCALE}")
    print()
    print("[ZEKO Body] Check the Viewport now.")
    print("[ZEKO Body] If the body looks misaligned, adjust TRANSLATE_X/Y/Z and re-run.")
    print("[ZEKO Body] Save the scene (Ctrl+S) once it looks correct.")


# ── Run when executed from Script Editor ────────────────────────────────────
add_body()
