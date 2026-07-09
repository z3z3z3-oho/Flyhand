#!/usr/bin/env python3
import yaml
import json

# 读取ost.yaml
with open("ost.yaml", "r") as f:
    calib = yaml.safe_load(f)

# 提取核心参数
intrinsics = {
    "camera_matrix": {
        "rows": 3,
        "cols": 3,
        "data": calib["camera_matrix"]["data"]
    },
    "distortion_coefficients": {
        "rows": 1,
        "cols": 5,
        "data": calib["distortion_coefficients"]["data"]
    },
    "image_width": calib["image_width"],
    "image_height": calib["image_height"]
}

# 保存为JSON
with open("camera_intrinsics.json", "w") as f:
    json.dump(intrinsics, f, indent=4)

print("✅ 相机内参已转换为 camera_intrinsics.json")
