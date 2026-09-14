#!/usr/bin/env python3
"""Convert rectified stereo PNG pairs into this project's replayable RGB-D layout.

The converter deliberately writes a new directory and leaves the source
dataset untouched.  Depth is computed in metres as Z = fx * baseline / d.
It must only be used with calibration from the exact camera that captured the
rectified image pairs.
"""

import argparse
import json
import os
import shutil
from pathlib import Path

import cv2
import numpy as np


def indexed_pngs(directory: Path):
    frames = {}
    for image in directory.glob("*.png"):
        try:
            frames[int(image.stem)] = image
        except ValueError:
            continue
    return frames


def make_matcher():
    block_size = 5
    channels = 1
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,
        blockSize=block_size,
        P1=8 * channels * block_size * block_size,
        P2=32 * channels * block_size * block_size,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def write_link_or_copy(source: Path, target: Path):
    if target.exists() or target.is_symlink():
        target.unlink()
    try:
        os.symlink(source.resolve(), target)
    except OSError:
        shutil.copy2(source, target)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Dataset containing left/ and right/")
    parser.add_argument("--output", required=True, help="New replay dataset directory")
    parser.add_argument("--calibration", required=True, help="camera_info.json from export_zed_calibration.py")
    parser.add_argument("--min-depth", type=float, default=0.3)
    parser.add_argument("--max-depth", type=float, default=10.0)
    args = parser.parse_args()

    source = Path(args.input).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    calibration = json.loads(Path(args.calibration).expanduser().read_text(encoding="utf-8"))
    left_frames = indexed_pngs(source / "left")
    right_frames = indexed_pngs(source / "right")
    frame_ids = sorted(set(left_frames).intersection(right_frames))
    if not frame_ids:
        raise RuntimeError("No matched numeric left/right PNG pairs were found")

    camera = calibration["left_camera"]
    fx = float(camera["fx"])
    baseline = float(calibration["stereo"]["baseline_m"])
    if fx <= 0.0 or baseline <= 0.0:
        raise RuntimeError("Calibration must contain positive fx and baseline_m")

    first = cv2.imread(str(left_frames[frame_ids[0]]), cv2.IMREAD_COLOR)
    if first is None:
        raise RuntimeError("Cannot read the first left image")
    height, width = first.shape[:2]
    expected = calibration.get("resolution", {})
    if expected and (width != expected.get("width") or height != expected.get("height")):
        raise RuntimeError(
            f"Image is {width}x{height}, but calibration is "
            f"{expected.get('width')}x{expected.get('height')}"
        )

    left_out = output / "left"
    depth_out = output / "depth_npy"
    left_out.mkdir(parents=True, exist_ok=True)
    depth_out.mkdir(parents=True, exist_ok=True)
    matcher = make_matcher()
    valid_ratios = []

    for output_index, frame_id in enumerate(frame_ids):
        left = cv2.imread(str(left_frames[frame_id]), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_frames[frame_id]), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None or left.shape != right.shape:
            raise RuntimeError(f"Invalid stereo pair at source frame {frame_id:04d}")

        disparity = matcher.compute(left, right).astype(np.float32) / 16.0
        depth = np.zeros_like(disparity, dtype=np.float32)
        valid = disparity > 0.5
        depth[valid] = (fx * baseline) / disparity[valid]
        valid &= np.isfinite(depth)
        valid &= depth >= args.min_depth
        valid &= depth <= args.max_depth
        depth[~valid] = 0.0
        valid_ratios.append(float(np.mean(valid)))

        stem = f"{output_index:06d}"
        write_link_or_copy(left_frames[frame_id], left_out / f"{stem}.png")
        np.save(depth_out / f"{stem}.npy", depth)

    replay_info = dict(calibration)
    replay_info["source_dataset"] = str(source)
    replay_info["source_frame_ids"] = frame_ids
    replay_info["depth_source"] = "StereoSGBM"
    replay_info["depth_unit"] = "metres"
    replay_info["valid_depth_ratio_mean"] = float(np.mean(valid_ratios))
    (output / "metadata.json").write_text(
        json.dumps({"total_frames": len(frame_ids)}, indent=2) + "\n", encoding="utf-8"
    )
    (output / "camera_info.json").write_text(
        json.dumps(replay_info, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Converted {len(frame_ids)} frames to {output}")
    print(f"Mean valid depth ratio: {np.mean(valid_ratios):.1%}")


if __name__ == "__main__":
    main()
