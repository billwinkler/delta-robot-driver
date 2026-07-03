"""Fit the pixel->mm map from the collect-data dataset.

The servo loop needs the JACOBIAN M (2x2, mm of arm motion per pixel of
pecan displacement). The intercept b is deliberately discarded: the
dataset shows the arm's dead-reckoned position drifts within a session
(labels wander by ~10-17 mm), so b is unreliable — and closed-loop
servoing replaces it with the calibrated grip-point pixel anyway.

Method: per-session RANSAC affine fits (peeling up to 2 clusters per
session, since drift splits sessions into consistent sub-epochs).
Clusters with enough inliers vote; the element-wise median of their
linear parts is the pooled Jacobian. Validation: per-cluster holdout
residuals using the pooled M.

Usage: python3 fit_affine.py [detections.csv]
"""

import csv
import sys
from collections import defaultdict

import numpy as np

TOL_MM = 3.0
MIN_CLUSTER = 8


def ransac_affine(P, O, rng, tol=TOL_MM, iters=4000):
    """P: (n,3) [px py 1]; O: (n,2) offsets. Returns (inlier_mask, A) or None."""
    n = len(P)
    if n < 4:
        return None
    best = None
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        try:
            A = np.linalg.solve(P[idx], O[idx])
        except np.linalg.LinAlgError:
            continue
        inl = np.linalg.norm(P @ A - O, axis=1) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 3:
        return None
    A, *_ = np.linalg.lstsq(P[best], O[best], rcond=None)
    inl = np.linalg.norm(P @ A - O, axis=1) < tol
    return inl, A


def main(csv_path):
    rng = np.random.default_rng(42)
    rows = [r for r in csv.DictReader(open(csv_path)) if r["px"]]
    sessions = defaultdict(list)
    for r in rows:
        sessions[r["image"].split("_sample_")[0]].append(r)

    jacobians, cluster_stats = [], []
    for name, rs in sorted(sessions.items()):
        P = np.array([[float(r["px"]), float(r["py"]), 1] for r in rs])
        O = np.array([[float(r["offset_x"]), float(r["offset_y"])] for r in rs])
        remaining = np.ones(len(P), bool)
        for peel in range(2):  # drift splits a session into ~2 epochs
            if remaining.sum() < MIN_CLUSTER:
                break
            res = ransac_affine(P[remaining], O[remaining], rng)
            if res is None:
                break
            inl, A = res
            if inl.sum() < MIN_CLUSTER:
                break
            resid = np.linalg.norm(P[remaining][inl] @ A - O[remaining][inl], axis=1)
            jacobians.append(A[:2])
            cluster_stats.append(
                {"session": name, "peel": peel, "n": int(inl.sum()),
                 "median_mm": float(np.median(resid)), "b": A[2].round(1).tolist()}
            )
            idx = np.where(remaining)[0]
            remaining[idx[inl]] = False

    for c in cluster_stats:
        print(f'{c["session"]} peel{c["peel"]}: n={c["n"]:3d} median={c["median_mm"]:.2f}mm b={c["b"]}')

    Ms = np.array(jacobians)  # (k, 2, 2) — rows are [px, py] coeffs per offset axis
    M = np.median(Ms, axis=0)
    spread = np.percentile(np.abs(Ms - M), 90, axis=0)
    print(f"\nclusters used: {len(Ms)} covering {sum(c['n'] for c in cluster_stats)} points")
    print("pooled Jacobian M (offset = [px py] @ M + b):")
    print(np.round(M, 4))
    print("p90 |element spread| across clusters:")
    print(np.round(spread, 4))
    scale = np.sqrt(np.abs(np.linalg.det(M)))
    theta = np.degrees(np.arctan2(M[0, 1], M[0, 0]))
    print(f"scale ~{scale:.3f} mm/px, rotation ~{theta:.1f} deg")

    # Validation: holdout within each cluster, using pooled M (only the
    # intercept re-fit from half the cluster — mirroring servo usage
    # where b comes from the grip-point calibration, not this dataset).
    holdout_resid = []
    for c in cluster_stats:
        rs = sessions[c["session"]]
        P = np.array([[float(r["px"]), float(r["py"])] for r in rs])
        O = np.array([[float(r["offset_x"]), float(r["offset_y"])] for r in rs])
        resid_full = np.linalg.norm(np.hstack([P, np.ones((len(P), 1))]) @
                                    np.vstack([M, [0, 0]]) - O, axis=1)
        # cluster membership: points whose residual (after best b) is small
        pred = P @ M
        errs = O - pred
        b_est = np.median(errs, axis=0)  # not quite cluster-specific; refine:
        member = np.linalg.norm(errs - b_est, axis=1) < TOL_MM * 2
        if member.sum() < MIN_CLUSTER:
            continue
        mp, mo = pred[member], O[member]
        half = len(mp) // 2
        b_train = np.median((mo - mp)[:half], axis=0)
        r_test = np.linalg.norm(mp[half:] + b_train - mo[half:], axis=1)
        holdout_resid.extend(r_test.tolist())
    hr = np.array(holdout_resid)
    print(f"\nholdout residuals (pooled M, intercept from first half): n={len(hr)}")
    print("mm: p50 %.2f  p90 %.2f  max %.2f" %
          (np.percentile(hr, 50), np.percentile(hr, 90), hr.max()))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "detector_eval/detections.csv")
