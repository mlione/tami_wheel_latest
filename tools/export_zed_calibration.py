#!/usr/bin/env python3
"""Export the calibration of the currently connected ZED camera as JSON.

Run this only while no other process owns the ZED camera.  The resulting file
is intended for stereo reconstruction of image pairs captured by that exact
camera at the selected resolution.
"""

import argparse
import json
from pathlib import Path

import pyzed.sl as sl


def translation_x(transform):
    translation = transform.get_translation()
    values = translation.get() if hasattr(translation, "get") else translation
    return float(values[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="Output camera_info.json path")
    parser.add_argument("--resolution", choices=("720", "1080", "2k", "vga"), default="720")
    args = parser.parse_args()

    resolutions = {
        "720": sl.RESOLUTION.HD720,
        "1080": sl.RESOLUTION.HD1080,
        "2k": sl.RESOLUTION.HD2K,
        "vga": sl.RESOLUTION.VGA,
    }
    init = sl.InitParameters()
    init.camera_resolution = resolutions[args.resolution]
    init.coordinate_units = sl.UNIT.METER

    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(
            f"Cannot open ZED ({status}). Stop the running ZED/Fusion launch first."
        )

    try:
        info = zed.get_camera_information()
        calibration = info.camera_configuration.calibration_parameters
        left = calibration.left_cam
        right = calibration.right_cam
        resolution = info.camera_configuration.resolution
        document = {
            "camera_model": str(info.camera_model),
            "serial_number": int(info.serial_number),
            "resolution": {"width": int(resolution.width), "height": int(resolution.height)},
            "left_camera": {
                "fx": float(left.fx), "fy": float(left.fy),
                "cx": float(left.cx), "cy": float(left.cy),
                "disto": [float(value) for value in left.disto],
            },
            "right_camera": {
                "fx": float(right.fx), "fy": float(right.fy),
                "cx": float(right.cx), "cy": float(right.cy),
                "disto": [float(value) for value in right.disto],
            },
            "stereo": {
                "baseline_m": abs(translation_x(calibration.stereo_transform)),
                "rectified_pairs_required": True,
            },
        }
    finally:
        zed.close()

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote calibration for ZED serial {document['serial_number']} to {output}")


if __name__ == "__main__":
    main()
