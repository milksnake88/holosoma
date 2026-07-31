"""demo_results npz (qpos) → PRISM MimicKit pkl ({loop_mode, fps, frames}).

frames 스키마 (prism motion_lib 소비 형식):
  [root_pos(3), root_rot expmap(3), dof(ROBOT_DOF, XML 선언 순서)]  — (T, 6+D)

안전장치: 저장 전 MJCF 모델의 관절 선언 순서가 EXPECTED_JOINT_ORDER 와
일치하는지 이름으로 검증한다 (dof 스크램블 사고 방지).

기본 동작: 앞/뒤 150프레임을 잘라낸다 (--trim-head / --trim-tail, 0=끔).
LAFAN 등 원본 mocap 이 T포즈로 시작·종료하므로 전환 구간을 학습 데이터에서 제외한다.

Usage:
    # 단일 파일
    python -m holosoma_retargeting.data_conversion.export_mimickit \
        --input demo_results/alice5/robot_only/lafan/walk1_subject1.npz \
        --output pkl/walk1_subject1.pkl
    # 디렉토리 일괄 (하위 폴더 재귀, 상대 구조 유지)
    python -m holosoma_retargeting.data_conversion.export_mimickit \
        --input demo_results/alice5/robot_only --output pkl/
"""

from __future__ import annotations

import pickle
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import mujoco  # type: ignore[import-not-found]
import numpy as np
import tyro
from scipy.spatial.transform import Rotation as sRot

src_root = Path(__file__).resolve().parents[2]
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))
from holosoma_retargeting.config_types.robot import RobotConfig  # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

LOOP_MODES = {"clamp": 0, "wrap": 1}

# PRISM 소비측(motion_lib)이 가정하는 dof 순서. 등록되지 않은 로봇은 검증을 건너뛴다.
EXPECTED_JOINT_ORDER: dict[str, list[str]] = {
    "alice5": [
        "waist_p", "waist_r", "waist_y", "head_y", "head_p",
        "l_sh_p", "l_sh_r", "l_el_p", "r_sh_p", "r_sh_r", "r_el_p",
        "l_hip_r", "l_hip_y", "l_hip_p", "l_knee_p", "l_ankle_p", "l_ankle_r",
        "r_hip_r", "r_hip_y", "r_hip_p", "r_knee_p", "r_ankle_p", "r_ankle_r",
    ],
}


def trim_qpos(qpos: np.ndarray, trim_head: int, trim_tail: int, min_frames: int = 30) -> np.ndarray:
    """qpos 앞/뒤 프레임을 잘라낸다 (원본 fps 기준 프레임 수).

    LAFAN 등 mocap 원본은 T포즈로 시작·종료하고 보행 진입까지 ~100프레임의
    전환 구간이 있다. 이 구간은 로봇 모션으로서 의미가 없고 학습 데이터
    품질을 떨어뜨리므로 제거한다. 저역통과(filtfilt)의 edge effect 가 전환
    구간의 큰 전이를 유효 구간으로 번지게 하므로 필터링보다 먼저 적용한다.
    """
    if trim_head < 0 or trim_tail < 0:
        raise ValueError(f"trim_head/trim_tail must be >= 0, got {trim_head}/{trim_tail}")
    if trim_head == 0 and trim_tail == 0:
        return qpos

    num_frames = qpos.shape[0]
    end = num_frames - trim_tail
    remaining = end - trim_head
    if remaining < min_frames:
        raise ValueError(
            f"trim_head={trim_head} + trim_tail={trim_tail} leaves {remaining} of "
            f"{num_frames} frames (minimum {min_frames})"
        )
    print(f"[export_mimickit] trim: {num_frames} → {remaining} frames (head {trim_head}, tail {trim_tail})")
    return qpos[trim_head:end]


def lowpass_qpos(qpos: np.ndarray, fps: float, cutoff_hz: float) -> np.ndarray:
    """qpos (T, 7+D) 제로위상 저역통과 (Butterworth 2차 filtfilt).

    프레임별 독립 최적화의 고주파 노이즈(떨림)를 제거한다. 사람 동작의
    에너지는 대부분 10Hz 이하라 12Hz 컷오프면 모션 왜곡 없이 떨림만 걸러진다.
    쿼터니언은 부호 연속화 후 rotvec 공간에서 필터링한다.
    """
    from scipy.signal import butter, filtfilt

    if cutoff_hz <= 0 or fps <= cutoff_hz * 2.5 or qpos.shape[0] < 30:
        return qpos
    b, a = butter(2, cutoff_hz / (fps / 2.0))
    out = qpos.copy()

    # root pos + dof: 직접 필터
    out[:, 0:3] = filtfilt(b, a, qpos[:, 0:3], axis=0)
    out[:, 7:] = filtfilt(b, a, qpos[:, 7:], axis=0)

    # root quat(wxyz): 부호 연속화 → rotvec 필터 → 복원
    q = qpos[:, 3:7].copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] *= -1.0
    rv = sRot.from_quat(q[:, [1, 2, 3, 0]]).as_rotvec()
    # rotvec 급전환(π 근처) 방어: 인접 차가 π/2 를 넘으면 필터를 건너뜀
    if np.abs(np.diff(rv, axis=0)).max() < np.pi / 2:
        rv = filtfilt(b, a, rv, axis=0)
        qf = sRot.from_rotvec(rv).as_quat()
        out[:, 3:7] = qf[:, [3, 0, 1, 2]]
    return out


def resample_qpos(qpos: np.ndarray, fps: float, target_fps: float) -> tuple[np.ndarray, float]:
    """qpos 를 target_fps 로 시간 리샘플 (pos/dof 선형, quat slerp).

    학습용 데이터의 표준 fps(30) 통일 + motion lib 메모리 절감용.
    출력 저역통과(기본 12Hz)가 선행되므로 30fps 데시메이션에서 앨리어싱 없음.
    """
    from scipy.spatial.transform import Slerp

    if target_fps <= 0 or abs(target_fps - fps) < 1e-6 or target_fps > fps:
        return qpos, fps
    num_frames = qpos.shape[0]
    t_src = np.arange(num_frames) / fps
    t_dst = np.arange(0.0, t_src[-1] + 1e-9, 1.0 / target_fps)

    out = np.empty((len(t_dst), qpos.shape[1]))
    for c in list(range(0, 3)) + list(range(7, qpos.shape[1])):
        out[:, c] = np.interp(t_dst, t_src, qpos[:, c])
    q = qpos[:, 3:7].copy()
    for i in range(1, num_frames):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] *= -1.0
    slerp = Slerp(t_src, sRot.from_quat(q[:, [1, 2, 3, 0]]))
    qs = slerp(np.clip(t_dst, t_src[0], t_src[-1])).as_quat()
    out[:, 3:7] = qs[:, [3, 0, 1, 2]]
    return out, target_fps


def verify_joint_order(robot_config: RobotConfig) -> tuple[list[str], np.ndarray]:
    """MJCF 의 hinge 관절 선언 순서 검증 + 관절 리밋 반환."""
    xml_path = PACKAGE_ROOT / robot_config.ROBOT_URDF_FILE.replace(".urdf", ".xml")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    names, ranges = [], []
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
            continue
        names.append(model.joint(i).name)
        ranges.append(model.jnt_range[i].copy())

    expected = EXPECTED_JOINT_ORDER.get(robot_config.robot_type)
    if expected is not None and names != expected:
        raise ValueError(
            f"joint order mismatch between {xml_path.name} and "
            f"EXPECTED_JOINT_ORDER['{robot_config.robot_type}']:\n"
            f"  model : {names}\n  expect: {expected}"
        )
    return names, np.asarray(ranges)


def export_mimickit_pkl(
    qpos: np.ndarray,
    fps: float,
    out_path: str | Path,
    robot_config: RobotConfig,
    loop_mode: str = "wrap",
) -> dict:
    """qpos (T, 7+D) [pos3, quat wxyz4, dof D] → MimicKit pkl 저장."""
    if loop_mode not in LOOP_MODES:
        raise ValueError(f"loop_mode must be one of {list(LOOP_MODES)}")
    qpos = np.asarray(qpos, dtype=np.float64)
    expected_dim = 7 + robot_config.ROBOT_DOF
    if qpos.ndim != 2 or qpos.shape[1] != expected_dim:
        raise ValueError(f"expected qpos (T, {expected_dim}), got {qpos.shape}")

    _, jnt_range = verify_joint_order(robot_config)

    root_pos = qpos[:, 0:3]
    quat_wxyz = qpos[:, 3:7]
    quat_wxyz = quat_wxyz / np.linalg.norm(quat_wxyz, axis=1, keepdims=True)
    expmap = sRot.from_quat(quat_wxyz[:, [1, 2, 3, 0]]).as_rotvec()
    # 저역통과 필터 등 후처리가 하드 리밋을 수 mrad 넘길 수 있음 → 최종 클램프
    dof = np.clip(qpos[:, 7:], jnt_range[:, 0], jnt_range[:, 1])

    frames = np.concatenate([root_pos, expmap, dof], axis=1).astype(np.float32)
    # float32 반올림이 f64 클램프 결과를 한계 밖으로 나노라디안 올릴 수 있다
    # → 한계를 f32 안쪽으로 조여 재클램프
    lo32 = jnt_range[:, 0].astype(np.float32)
    hi32 = jnt_range[:, 1].astype(np.float32)
    lo32 = np.where(lo32.astype(np.float64) < jnt_range[:, 0], np.nextafter(lo32, np.float32(np.inf)), lo32)
    hi32 = np.where(hi32.astype(np.float64) > jnt_range[:, 1], np.nextafter(hi32, np.float32(-np.inf)), hi32)
    frames[:, 6:] = np.clip(frames[:, 6:], lo32, hi32)

    out = {
        "loop_mode": LOOP_MODES[loop_mode],
        "fps": int(round(float(fps))),
        "frames": frames.tolist(),
    }
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        pickle.dump(out, f)
    print(f"[export_mimickit] saved {out_path}: frames={frames.shape}, fps={out['fps']}, loop={loop_mode}")
    return out


@dataclass(frozen=True)
class ExportMimickitConfig:
    """demo_results npz → MimicKit pkl 변환 설정."""

    input: Path
    """입력 npz 파일 또는 디렉토리 (디렉토리는 하위 폴더까지 재귀 수집)."""
    output: Path
    """출력 pkl 파일 (입력이 파일일 때) 또는 디렉토리 (입력이 디렉토리일 때)."""
    robot_type: str = "alice5"
    """로봇 타입 — dof 수/모델 XML(관절 순서·리밋 검증) 결정."""
    loop_mode: Literal["wrap", "clamp"] = "wrap"
    """모션 반복 모드."""
    fps: float | None = None
    """fps 오버라이드 (기본: npz 의 fps, 없으면 30)."""
    trim_head: int = 150
    """앞에서 제거할 프레임 수 (입력 fps 기준). LAFAN 원본은 T포즈로 시작해 보행 진입까지 ~100프레임이 걸린다."""
    trim_tail: int = 150
    """뒤에서 제거할 프레임 수 (입력 fps 기준). 원본이 T포즈로 종료되는 구간 제거."""
    output_filter_hz: float = 0.0
    """출력 저역통과 컷오프(Hz, 제로위상). 0=끔. 이미 후처리된 결과면 끄는 게 기본."""
    resample_fps: float = 0.0
    """출력 리샘플 fps. 0=원본 유지."""


def convert_one(in_path: Path, out_path: Path, cfg: ExportMimickitConfig, robot_config: RobotConfig) -> None:
    """npz 하나를 pkl 하나로 변환한다."""
    data = np.load(str(in_path))
    if "qpos" not in data:
        raise KeyError(f"{in_path}: 'qpos' key not found (keys={list(data.files)})")
    qpos = np.asarray(data["qpos"], dtype=np.float64)
    fps = float(cfg.fps) if cfg.fps is not None else float(data["fps"]) if "fps" in data.files else 30.0

    qpos = trim_qpos(qpos, cfg.trim_head, cfg.trim_tail)
    if cfg.output_filter_hz > 0:
        qpos = lowpass_qpos(qpos, fps, cfg.output_filter_hz)
    if cfg.resample_fps > 0:
        qpos, fps = resample_qpos(qpos, fps, cfg.resample_fps)
    export_mimickit_pkl(qpos, fps, out_path, robot_config, loop_mode=cfg.loop_mode)


def main(cfg: ExportMimickitConfig) -> None:
    """단일 파일 또는 디렉토리를 일괄 변환한다."""
    robot_config = RobotConfig(robot_type=cfg.robot_type)

    if cfg.input.is_dir():
        npz_files = sorted(cfg.input.rglob("*.npz"))
        if not npz_files:
            raise FileNotFoundError(f"no .npz under {cfg.input}")
        jobs = [(p, cfg.output / p.relative_to(cfg.input).with_suffix(".pkl")) for p in npz_files]
    else:
        out = cfg.output / f"{cfg.input.stem}.pkl" if cfg.output.suffix != ".pkl" else cfg.output
        jobs = [(cfg.input, out)]

    n_ok = 0
    for in_path, out_path in jobs:
        try:
            convert_one(in_path, out_path, cfg, robot_config)
            n_ok += 1
        except Exception as e:  # noqa: BLE001 — 한 파일 실패가 일괄 변환을 막지 않게
            print(f"[export_mimickit] FAILED {in_path}: {type(e).__name__}: {e}")
    print(f"[export_mimickit] done: {n_ok}/{len(jobs)} converted")


if __name__ == "__main__":
    main(tyro.cli(ExportMimickitConfig))
