"""viser_player 재생 화면에서 구간을 잘라 npz 클립으로 저장하는 GUI.

단일 모션 파일에 원하는 동작(예: walking)과 원치 않는 동작(예: recovery)이
섞여 있을 때, 재생 슬라이더로 경계를 찾아 필요한 구간만 데이터셋으로 뽑는다.

start/end 는 재생 슬라이더의 현재 프레임을 그대로 집어오며(end 포함),
저장물은 `{원본stem}_{start}-{end}.npz` 다. 원본이 npz 면 프레임 축을 공유하는
모든 배열(human_joints 등)을 같은 구간으로 함께 잘라 담으므로 클립만으로도
재리타겟·평가가 가능하다. 저장된 클립은 export_mimickit.py 로 그대로 pkl 변환.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import viser  # type: ignore[import-not-found]


def add_clip_gui(
    server: viser.ViserServer,
    frame_slider: viser.GuiInputHandle,
    qpos: np.ndarray,
    fps: int,
    source: str | Path,
    out_dir: str | Path | None = None,
) -> None:
    """재생 중인 모션에서 [start, end] 구간을 npz 로 저장하는 GUI 를 추가한다.

    Args:
        server: viser 서버.
        frame_slider: create_motion_control_sliders 가 리턴한 프레임 슬라이더.
        qpos: 플레이어가 재생 중인 qpos (T, 7+D).
        fps: 클립에 기록할 fps.
        source: 재생 중인 원본 파일 경로 (npz 또는 pkl) — 클립 이름·부가 배열의 출처.
        out_dir: 저장 디렉토리. None 이면 `<원본 디렉토리>_clips` (원본과 섞이지 않게).
    """
    src = Path(source)
    n_frames = int(qpos.shape[0])
    clip_dir = Path(out_dir) if out_dir else src.parent.parent / f"{src.parent.name}_clips"

    # 원본이 npz 면 qpos 외의 배열(human_joints 등)도 함께 잘라 보존한다.
    extras: dict[str, np.ndarray] = {}
    if src.suffix == ".npz" and src.exists():
        with np.load(src, allow_pickle=True) as data:
            extras = {k: np.asarray(data[k]) for k in data.files}

    def _auto_name(start: int, end: int) -> str:
        return f"{src.stem}_{start}-{end}"

    with server.gui.add_folder("Clip export"):
        start_in = server.gui.add_number("Start", initial_value=0, min=0, max=n_frames - 1, step=1)
        end_in = server.gui.add_number("End", initial_value=n_frames - 1, min=0, max=n_frames - 1, step=1)
        set_start_btn = server.gui.add_button("Set start = current frame")
        set_end_btn = server.gui.add_button("Set end = current frame")
        name_in = server.gui.add_text("Name", initial_value=_auto_name(0, n_frames - 1))
        save_btn = server.gui.add_button("Save clip (npz)")
        status = server.gui.add_markdown(f"out: `{clip_dir}`")

    # 사용자가 이름을 직접 고쳤으면 자동 갱신을 멈춘다.
    auto = {"name": name_in.value}

    def _refresh_name() -> None:
        if name_in.value != auto["name"]:
            return
        auto["name"] = _auto_name(int(start_in.value), int(end_in.value))
        name_in.value = auto["name"]

    @set_start_btn.on_click
    def _(_evt) -> None:
        start_in.value = int(frame_slider.value)
        _refresh_name()

    @set_end_btn.on_click
    def _(_evt) -> None:
        end_in.value = int(frame_slider.value)
        _refresh_name()

    @start_in.on_update
    def _(_evt) -> None:
        _refresh_name()

    @end_in.on_update
    def _(_evt) -> None:
        _refresh_name()

    @save_btn.on_click
    def _(_evt) -> None:
        start, end = int(start_in.value), int(end_in.value)
        if not 0 <= start < end < n_frames:
            msg = f"invalid range [{start}, {end}] for {n_frames} frames"
            print(f"[clip_motion] {msg}")
            status.content = f"**error**: {msg}"
            return

        # 프레임 축을 공유하는 배열만 자르고, 스칼라 메타(fps, cost 등)는 그대로 둔다.
        payload = {k: (v[start : end + 1] if v.ndim >= 1 and v.shape[0] == n_frames else v) for k, v in extras.items()}
        payload["qpos"] = qpos[start : end + 1]
        payload["fps"] = np.asarray(int(fps))

        name = name_in.value.strip() or _auto_name(start, end)
        out_path = clip_dir / f"{Path(name).stem}.npz"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(out_path, **payload)

        n_clip = end + 1 - start
        print(f"[clip_motion] saved {out_path}: frames={n_clip} ({start}-{end}), fps={int(fps)}")
        status.content = f"saved `{out_path.name}` — {n_clip} frames ({start}-{end})\n\nout: `{clip_dir}`"
