#!/usr/bin/env python3
"""Generate an OpenVINS kalibr_imucam_chain.yaml from a UZH-FPV Kalibr camchain.

OpenVINS reads the Kalibr chain natively, BUT via OpenCV's cv::FileStorage YAML
parser, which is far stricter than PyYAML about indentation: block-sequence items
must be indented *under* their key (4 spaces), not aligned with it (the 2-space
style PyYAML/Kalibr emit, which cv::FileStorage rejects with "Incorrect
indentation"). This emits the exact layout OpenVINS' working configs use.

T_cam_imu / T_cn_cnm1 are passed through verbatim (OpenVINS uses the same Kalibr
convention — no inversion). The IMU chain (kalibr_imu_chain.yaml) is the same
physical sensor across UZH rigs, so reuse an existing one.

Usage:
  uzh_calib_to_openvins.py <camchain-imucam-*.yaml> <out_kalibr_imucam_chain.yaml>
"""
import argparse
import sys

import yaml


def fmt_mat(rows):
    return "".join(f"    - [{', '.join(repr(float(v)) for v in r)}]\n" for r in rows)


def fmt_list(vals):
    return "[" + ", ".join(repr(float(v)) for v in vals) + "]"


def cam_block(name, c, with_cn):
    out = [f"{name}:"]
    out.append("  T_cam_imu:")
    out.append(fmt_mat(c["T_cam_imu"]).rstrip("\n"))
    if with_cn:
        out.append("  T_cn_cnm1:")
        out.append(fmt_mat(c["T_cn_cnm1"]).rstrip("\n"))
    out.append(f"  cam_overlaps: {fmt_list(c.get('cam_overlaps', []))}"
               .replace(".0]", "]").replace(".0,", ","))
    out.append(f"  camera_model: {c['camera_model']}")
    out.append(f"  distortion_coeffs: {fmt_list(c['distortion_coeffs'])}")
    out.append(f"  distortion_model: {c['distortion_model']}")
    out.append(f"  intrinsics: {fmt_list(c['intrinsics'])}")
    out.append(f"  resolution: {fmt_list(c['resolution'])}"
               .replace(".0]", "]").replace(".0,", ","))
    out.append(f"  rostopic: {c['rostopic']}")
    if "timeshift_cam_imu" in c:
        out.append(f"  timeshift_cam_imu: {float(c['timeshift_cam_imu'])!r}")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("camchain")
    ap.add_argument("out")
    args = ap.parse_args()

    cc = yaml.safe_load(open(args.camchain))
    cams = [k for k in sorted(cc) if k.startswith("cam")]
    if not cams:
        sys.exit(f"ERROR: no cam* blocks in {args.camchain}")

    parts = ["%YAML:1.0 # need to specify the file type at the top!", ""]
    for i, cam in enumerate(cams):
        parts.append(cam_block(cam, cc[cam], with_cn=(i > 0)))
    with open(args.out, "w") as f:
        f.write("\n".join(parts) + "\n")
    print(f"wrote {args.out}: {len(cams)} cams {cams} "
          f"({cc[cams[0]]['distortion_model']}, cv::FileStorage layout)")


if __name__ == "__main__":
    main()
