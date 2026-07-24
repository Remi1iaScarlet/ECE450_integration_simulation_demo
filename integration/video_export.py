"""Full-resolution MP4 export of a bridge run, for recording a demo video.

visual_grasp's own evidence saver (multitask/executor.py:_save_evidence)
writes a downscaled, frame-capped GIF -- good for quick in-repo evidence, too
low quality to film off a screen. This module is a drop-in replacement
passed as `evidence_saver=` to `bridge.run_bridge()` / `execute_command()`;
it does not modify anything under visual_grasp/.
"""
from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

EVIDENCE_DIR = Path(__file__).resolve().parent.parent / "visual_grasp" / "multitask" / "evidence"


def mp4_evidence_saver(capture_fps: int = 15, output_fps: int = 30):
    """Return an evidence_saver(world, name) that writes name.mp4 at full
    (640x480) resolution instead of the repo default's downscaled GIF.

    capture_fps should roughly match how often world.py appends a frame
    (every `every` sim steps / control timestep); it only affects playback
    speed, not what gets captured. output_fps just controls smoothness via
    ffmpeg's fps filter (duplicating/dropping frames), not real information.
    """

    def save(world, name: str) -> None:
        from PIL import Image

        EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
        if not world.frames:
            print(f"[video_export] {name}: no frames captured, nothing to export")
            return

        with tempfile.TemporaryDirectory() as tmp:
            for i, arr in enumerate(world.frames):
                Image.fromarray(arr).save(f"{tmp}/frame_{i:05d}.png")

            out_path = EVIDENCE_DIR / f"{name}.mp4"
            cmd = [
                "ffmpeg", "-y",
                "-framerate", str(capture_fps),
                "-i", f"{tmp}/frame_%05d.png",
                "-vf", f"fps={output_fps},format=yuv420p",
                "-c:v", "libx264", "-crf", "18", "-preset", "medium",
                str(out_path),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print(f"[video_export] ffmpeg failed for {name}:\n{result.stderr[-2000:]}")
                return
            print(f"[video_export] saved {out_path} ({len(world.frames)} frames -> {output_fps}fps)")

    return save
