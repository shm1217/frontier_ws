#!/usr/bin/env python3

import cv2
import yaml

input_yaml = "tb3_0_map.yaml"
output_yaml = "tb3_0_map_rotated.yaml"
output_pgm = "tb3_0_map_rotated.pgm"

with open(input_yaml, "r") as f:
    metadata = yaml.safe_load(f)

image = cv2.imread(metadata["image"], cv2.IMREAD_GRAYSCALE)

if image is None:
    raise RuntimeError("PGM 파일을 읽지 못했습니다.")

resolution = float(metadata["resolution"])
origin_x = float(metadata["origin"][0])
origin_y = float(metadata["origin"][1])

height, width = image.shape

# 이미지 Y축은 아래 방향이므로,
# 물리 map 기준 반시계 90도는 이미지 기준 시계 90도입니다.
rotated = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

new_height, new_width = rotated.shape

# 회전 전후 map 중심 위치를 동일하게 유지
center_x = origin_x + width * resolution / 2.0
center_y = origin_y + height * resolution / 2.0

new_origin_x = center_x - new_width * resolution / 2.0
new_origin_y = center_y - new_height * resolution / 2.0

cv2.imwrite(output_pgm, rotated)

metadata["image"] = output_pgm
metadata["origin"] = [
    new_origin_x,
    new_origin_y,
    0.0,
]

with open(output_yaml, "w") as f:
    yaml.safe_dump(metadata, f, sort_keys=False)

print(f"저장 완료: {output_pgm}")
print(f"저장 완료: {output_yaml}")
