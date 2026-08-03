#!/usr/bin/env python3
# viser_player.py
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import tyro
import viser  # type: ignore[import-not-found]  # pip install viser
import yourdfpy  # type: ignore[import-untyped]  # pip install yourdfpy
from viser.extras import ViserUrdf  # type: ignore[import-not-found]

src_root = Path(__file__).resolve().parent.parent
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))
from holosoma_retargeting.config_types.viser import ViserConfig  # noqa: E402
from holosoma_retargeting.data_conversion.clip_motion import add_clip_gui  # noqa: E402
from holosoma_retargeting.src.viser_utils import create_motion_control_sliders  # noqa: E402


def load_npz(npz_path: str):
    data = np.load(npz_path, allow_pickle=True)
    # expected: qpos [T, ?], and optional fps
    qpos = data["qpos"]
    fps = int(data["fps"]) if "fps" in data else 30
    return qpos, fps


def load_mimickit_pkl(pkl_path: str):
    """MimicKit pkl → (qpos, fps), i.e. the inverse of export_mimickit.py.

    pkl frames are `[root_pos(3), root_rot expmap(3), dof(D)]` (T, 6+D) while the
    player consumes qpos `[pos(3), quat wxyz(4), dof(D)]`, so only the
    expmap(rotvec) → quaternion conversion is needed.
    """
    import pickle

    from scipy.spatial.transform import Rotation as sRot

    with open(pkl_path, "rb") as f:
        data = pickle.load(f)
    frames = np.asarray(data["frames"], dtype=np.float64)
    if frames.ndim != 2 or frames.shape[1] < 7:
        raise ValueError(f"unexpected frames shape {frames.shape} in {pkl_path}")
    quat_xyzw = sRot.from_rotvec(frames[:, 3:6]).as_quat()
    qpos = np.concatenate([frames[:, 0:3], quat_xyzw[:, [3, 0, 1, 2]], frames[:, 6:]], axis=1)
    return qpos, int(round(float(data.get("fps", 30))))


def make_player(
    config: ViserConfig,
    qpos: np.ndarray,
    fps: int | None = None,
):
    """
    qpos layout (MuJoCo order):
      [0:3]   robot base position (xyz)
      [3:7]   robot base quat (wxyz)
      [7:7+R] robot joint positions (R = actuated dof)
      [end-7:end-4] (optional) object position (xyz)
      [end-4:end]   (optional) object quat (wxyz)

    We'll infer R from the robot URDF's actuated joints in ViserUrdf.
    """
    server = viser.ViserServer()

    # Root frames
    robot_root = server.scene.add_frame("/robot", show_axes=False)
    object_root = server.scene.add_frame("/object", show_axes=False)

    # URDFs (using yourdfpy so meshes show up)
    robot_urdf_y = yourdfpy.URDF.load(config.robot_urdf, load_meshes=True, build_scene_graph=True)
    vr = ViserUrdf(server, urdf_or_path=robot_urdf_y, root_node_name="/robot")

    vo = None
    if config.object_urdf:
        object_urdf_y = yourdfpy.URDF.load(config.object_urdf, load_meshes=True, build_scene_graph=True)
        vo = ViserUrdf(server, urdf_or_path=object_urdf_y, root_node_name="/object")

    # A tiny grid
    server.scene.add_grid("/grid", width=config.grid_width, height=config.grid_height, position=(0.0, 0.0, 0.0))

    # Figure robot DOF from actuated limits in ViserUrdf
    joint_limits = vr.get_actuated_joint_limits()
    robot_dof = len(joint_limits)

    # Use fps from config if not provided, otherwise use the one from npz file
    actual_fps = fps if fps is not None else config.fps

    # Set initial mesh visibility
    vr.show_visual = config.show_meshes
    if vo is not None:
        vo.show_visual = config.show_meshes

    # ---------- Additional GUI controls (mesh visibility) ----------
    with server.gui.add_folder("Display"):
        show_meshes_cb = server.gui.add_checkbox("Show meshes", initial_value=config.show_meshes)

    @show_meshes_cb.on_update
    def _(_):
        vr.show_visual = bool(show_meshes_cb.value)
        if vo is not None:
            vo.show_visual = bool(show_meshes_cb.value)

    # ---------- Use reusable motion control sliders from viser_utils ----------
    controls, _ = create_motion_control_sliders(
        server=server,
        viser_robot=vr,
        robot_base_frame=robot_root,
        motion_sequence=qpos,
        robot_dof=robot_dof,
        viser_object=vo if config.assume_object_in_qpos else None,
        object_base_frame=object_root if config.assume_object_in_qpos else None,
        contains_object_in_qpos=config.assume_object_in_qpos,
        initial_fps=actual_fps,
        initial_interp_mult=config.visual_fps_multiplier,
        loop=config.loop,
    )

    # ---------- Clip export (start/end 구간만 npz 로 저장) ----------
    add_clip_gui(
        server=server,
        frame_slider=controls[0],
        qpos=qpos,
        fps=actual_fps,
        source=config.mimickit_pkl or config.qpos_npz,
        out_dir=config.clip_out_dir,
    )

    n_frames = int(qpos.shape[0])
    print(
        f"[viser_player] Loaded {n_frames} frames | robot_dof={robot_dof} | "
        f"object={'yes' if (config.object_urdf and config.assume_object_in_qpos) else 'no'}"
    )
    print("Open the viewer URL printed above. Close the process (Ctrl+C) to exit.")
    return server


def main(cfg: ViserConfig) -> None:
    """Main function for viser player."""
    qpos, fps = load_mimickit_pkl(cfg.mimickit_pkl) if cfg.mimickit_pkl else load_npz(cfg.qpos_npz)
    make_player(
        config=cfg,
        qpos=qpos,
        fps=fps,
    )

    # keep process alive
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    cfg = tyro.cli(ViserConfig)
    main(cfg)
