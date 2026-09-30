#!/usr/bin/env python3
"""Compare timestamp-associated SDK DEPTH_MAP/POINT3D without assuming depth semantics.

Offline diagnostic only. Numeric agreement is not physical metric validation.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def load_pair(path):
    meta = json.loads(path.read_text())
    if meta['timestamp_ns'] != meta['point3d_timestamp_ns']:
        raise ValueError(f'{path}: SDK timestamps differ')
    h, w = meta['height'], meta['width']
    if not isinstance(h, int) or not isinstance(w, int) or min(h, w) <= 0:
        raise ValueError(f'{path}: invalid dimensions')
    dtype = np.dtype('>f4' if meta['is_bigendian'] else '<f4')

    def read(key, stride_key, channels):
        stride = meta[stride_key]
        data = (path.parent / meta[key]).read_bytes()
        if stride < 4 * w * channels or len(data) != stride * h:
            raise ValueError(f'{path}: invalid {key} payload/stride')
        return np.ndarray((h, w, channels), dtype=dtype, buffer=data,
                          strides=(stride, channels * 4, 4)).astype(np.float64)

    return meta, read('depth_file', 'depth_stride', 1)[:, :, 0], read('xyz_file', 'xyz_stride', 3)


def distribution(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if not values.size:
        return None
    return {k: float(v) for k, v in zip(('min', 'p05', 'median', 'p95', 'max'),
                                      np.percentile(values, [0, 5, 50, 95, 100]))}


def error_stats(error):
    error = np.asarray(error)
    error = error[np.isfinite(error)]
    if not error.size:
        return {'samples': 0}
    return {'samples': int(error.size), 'median_abs': float(np.median(abs(error))),
            'p95_abs': float(np.percentile(abs(error), 95)),
            'rmse': float(np.sqrt(np.mean(error ** 2)))}


def analyze(paths, camera_info=None, roi=None):
    counts = {k: 0 for k in ('pixels', 'nan', 'positive_inf', 'negative_inf', 'zero',
                             'negative_finite', 'positive_finite', 'xyz_nonfinite',
                             'xyz_zero', 'positive_depth_invalid_xyz')}
    errors = {k: [] for k in ('depth_minus_sdk_z', 'depth_minus_sdk_norm',
                             'pinhole_xyz_error', 'unit_ray_xyz_error')}
    ratios_z, ratios_norm, depths, center_roi = [], [], [], []
    pixel_samples, frames = [], []
    for index, path in enumerate(paths):
        meta, d, xyz = load_pair(path)
        h, w = d.shape
        norm = np.linalg.norm(xyz, axis=2)
        valid_d = np.isfinite(d) & (d > 0)
        finite_xyz = np.isfinite(xyz).all(axis=2)
        zero_xyz = (xyz == 0).all(axis=2)
        valid = valid_d & finite_xyz & ~zero_xyz
        counts['pixels'] += int(d.size)
        for key, mask in (('nan', np.isnan(d)), ('positive_inf', np.isposinf(d)),
                          ('negative_inf', np.isneginf(d)), ('zero', d == 0),
                          ('negative_finite', np.isfinite(d) & (d < 0)),
                          ('positive_finite', valid_d), ('xyz_nonfinite', ~finite_xyz),
                          ('xyz_zero', zero_xyz),
                          ('positive_depth_invalid_xyz', valid_d & (~finite_xyz | zero_xyz))):
            counts[key] += int(np.count_nonzero(mask))
        depths.append(d[valid_d])
        errors['depth_minus_sdk_z'].append(d[valid] - xyz[:, :, 2][valid])
        errors['depth_minus_sdk_norm'].append(d[valid] - norm[valid])
        nonzero_z = valid & (np.abs(xyz[:, :, 2]) > 1e-9)
        ratios_z.append(d[nonzero_z] / xyz[:, :, 2][nonzero_z])
        positive_norm = valid & (norm > 1e-9)
        ratios_norm.append(d[positive_norm] / norm[positive_norm])
        if camera_info:
            if camera_info['width'] != w or camera_info['height'] != h:
                raise ValueError('CameraInfo dimensions differ from SDK frames')
            k = np.asarray(camera_info['k']).reshape(3, 3)
            if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
                raise ValueError('Invalid CameraInfo K')
            v, u = np.indices((h, w))
            rays = np.stack(((u-k[0, 2])/k[0, 0], (v-k[1, 2])/k[1, 1], np.ones_like(d)), axis=2)
            valid_rays = rays[valid]
            reconstructed_z = valid_rays * d[valid, None]
            reconstructed_norm = valid_rays / np.linalg.norm(valid_rays, axis=1)[:, None] * d[valid, None]
            errors['pinhole_xyz_error'].append(np.linalg.norm(reconstructed_z - xyz[valid], axis=1))
            errors['unit_ray_xyz_error'].append(np.linalg.norm(reconstructed_norm - xyz[valid], axis=1))
        if roi:
            x, y, rw, rh = roi
            if min(x, y) < 0 or min(rw, rh) <= 0 or x + rw > w or y + rh > h:
                raise ValueError('ROI outside frame')
            crop = d[y:y+rh, x:x+rw]
            center_roi.append(crop[np.isfinite(crop) & (crop > 0)])
        if index in (0, len(paths) - 1):
            for v in np.linspace(0.1*h, 0.9*h-1, 5, dtype=int):
                for u in np.linspace(0.1*w, 0.9*w-1, 5, dtype=int):
                    if valid[v, u]:
                        pixel_samples.append({'timestamp_ns': meta['timestamp_ns'], 'u': int(u), 'v': int(v),
                                              'depth': float(d[v, u]), 'xyz': xyz[v, u].tolist(),
                                              'sdk_norm': float(norm[v, u])})
        frames.append({'timestamp_ns': meta['timestamp_ns'], 'width': w, 'height': h,
                       'positive_depth': distribution(d[valid_d])})

    def concat(arrays):
        return np.concatenate(arrays) if arrays else np.array([])

    return {'paired_frames': len(paths), 'invalid_counts': counts,
            'positive_depth_distribution': distribution(concat(depths)),
            'candidate_errors': {k: error_stats(concat(v)) for k, v in errors.items()},
            'depth_over_sdk_z': distribution(concat(ratios_z)),
            'depth_over_sdk_norm': distribution(concat(ratios_norm)),
            'roi': roi, 'roi_positive_depth': distribution(concat(center_roi)),
            'sample_pixels': pixel_samples, 'frames': frames,
            'physical_metric_validation': 'PENDING: independent measured surfaces at 0.5/1.0/1.5/2.0 m',
            'units_note': 'Raw SDK numeric units. Agreement with XYZ alone does not establish metres or accuracy.',
            'phase2_pass': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_dir', type=Path)
    parser.add_argument('--ros-probe-json', type=Path)
    parser.add_argument('--roi', nargs=4, type=int, metavar=('X', 'Y', 'WIDTH', 'HEIGHT'))
    args = parser.parse_args()
    paths = sorted(args.capture_dir.glob('*.json'))
    if not paths:
        parser.error('No paired-frame metadata found')
    info = json.loads(args.ros_probe_json.read_text())['last_camera_info'] if args.ros_probe_json else None
    try:
        print(json.dumps(analyze(paths, info, args.roi), indent=2, allow_nan=False))
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
