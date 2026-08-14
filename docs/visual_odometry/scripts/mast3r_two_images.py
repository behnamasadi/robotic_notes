#!/usr/bin/env python
"""Two images -> MASt3R geometry: pointmaps, depth, matches, focal, relative pose,
and a colored point cloud (PLY).

Usage:
    python mast3r_two_images.py img1.jpg img2.jpg [--out cloud.ply] [--conf-keep 0.7] [--rerun]

--rerun spawns the Rerun viewer with the fused cloud and both camera frustums.

Companion to docs/visual_odometry/dust3r_mast3r.ipynb, which explains every step.
"""
import argparse
import sys
from pathlib import Path

MAST3R_ROOT = Path(__file__).resolve().parents[1] / "third_party" / "mast3r"
sys.path.insert(0, str(MAST3R_ROOT))
sys.path.insert(0, str(MAST3R_ROOT / "dust3r"))
sys.path.insert(0, str(MAST3R_ROOT / "dust3r" / "croco"))  # croco's `models` package

import cv2
import numpy as np
import torch
import trimesh

from mast3r.model import AsymmetricMASt3R
from mast3r.fast_nn import fast_reciprocal_NNs
from dust3r.inference import inference
from dust3r.utils.image import load_images
from dust3r.post_process import estimate_focal_knowing_depth


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("img1")
    ap.add_argument("img2")
    ap.add_argument("--out", default="mast3r_pair.ply", help="output point cloud")
    ap.add_argument("--conf-keep", type=float, default=0.7,
                    help="fraction of most-confident points to keep (0..1]")
    ap.add_argument("--rerun", action="store_true", help="spawn the Rerun viewer")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = AsymmetricMASt3R.from_pretrained(
        "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric").to(device).eval()

    images = load_images([args.img1, args.img2], size=512)
    output = inference([tuple(images)], model, device, batch_size=1, verbose=False)
    pred1, pred2 = output["pred1"], output["pred2"]

    # pointmaps: X^{1,1} and X^{2,1} — BOTH in camera-1 frame
    pts3d_1 = pred1["pts3d"].squeeze(0).detach().cpu().numpy()
    pts3d_2 = pred2["pts3d_in_other_view"].squeeze(0).detach().cpu().numpy()
    conf_1 = pred1["conf"].squeeze(0).detach().cpu().numpy()
    conf_2 = pred2["conf"].squeeze(0).detach().cpu().numpy()
    H, W = pts3d_1.shape[:2]

    # descriptor matching (MASt3R's contribution)
    matches_1, matches_2 = fast_reciprocal_NNs(
        pred1["desc"].squeeze(0).detach(), pred2["desc"].squeeze(0).detach(),
        subsample_or_initxy1=8, device=device, dist="dot", block_size=2**13)
    print(f"matches: {len(matches_1)}")

    # focal via Weiszfeld on X^{1,1}
    pp = torch.tensor([W / 2, H / 2])
    focal = float(estimate_focal_knowing_depth(
        pred1["pts3d"].detach().cpu(), pp, focal_mode="weiszfeld"))
    K = np.array([[focal, 0, W / 2], [0, focal, H / 2], [0, 0, 1]])
    print(f"focal: {focal:.1f} px (at {W}x{H})")

    # relative pose via PnP: pixels of image 2 <-> X^{2,1}
    sel = np.where(conf_2.ravel() > np.quantile(conf_2, 0.5))[0][::20]
    uv = np.stack(np.meshgrid(np.arange(W), np.arange(H)), -1).reshape(-1, 2).astype(np.float64)
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        pts3d_2.reshape(-1, 3).astype(np.float64)[sel], uv[sel], K, None,
        flags=cv2.SOLVEPNP_SQPNP, iterationsCount=300, reprojectionError=3.0)
    R_w2c, _ = cv2.Rodrigues(rvec)
    R, t = R_w2c.T, (-R_w2c.T @ tvec).ravel()
    angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    print(f"camera 2 in cam-1 frame: t = {t.round(3)} m, rotation = {angle:.1f} deg")

    # fused colored cloud, confidence-filtered
    rgb = [((im["img"].squeeze(0).permute(1, 2, 0).numpy() + 1) / 2) for im in images]
    pts = np.concatenate([pts3d_1.reshape(-1, 3), pts3d_2.reshape(-1, 3)])
    cols = np.concatenate([rgb[0].reshape(-1, 3), rgb[1].reshape(-1, 3)])
    conf = np.concatenate([conf_1.ravel(), conf_2.ravel()])
    mask = conf > np.quantile(conf, 1 - args.conf_keep)
    trimesh.PointCloud(pts[mask], colors=(cols[mask] * 255).astype(np.uint8)).export(args.out)
    print(f"wrote {args.out} ({mask.sum():,} points) — open in MeshLab/CloudCompare")

    if args.rerun:
        import rerun as rr
        rr.init("mast3r_two_images", spawn=True)
        rr.log("world", rr.ViewCoordinates.RDF, static=True)
        rr.log("world/points", rr.Points3D(pts[mask], colors=(cols[mask] * 255).astype(np.uint8)))
        for name, Rc, tc, im in [("cam1", np.eye(3), np.zeros(3), rgb[0]),
                                 ("cam2", R, t, rgb[1])]:
            rr.log(f"world/{name}", rr.Transform3D(translation=tc, mat3x3=Rc))
            rr.log(f"world/{name}/image", rr.Pinhole(focal_length=focal, width=W, height=H))
            rr.log(f"world/{name}/image", rr.Image((im * 255).astype(np.uint8)))


if __name__ == "__main__":
    main()
