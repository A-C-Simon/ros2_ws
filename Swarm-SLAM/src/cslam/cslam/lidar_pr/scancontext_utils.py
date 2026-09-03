# Adapted from https://github.com/gisbi-kim/PyICP-SLAM/blob/master/utils/ScanContextManager.py

import numpy as np
np.set_printoptions(precision=4)

import time
from scipy import spatial


def xy2theta(x, y):
    if (x >= 0 and y >= 0): 
        theta = 180/np.pi * np.arctan(y/x)
    elif (x < 0 and y >= 0): 
        theta = 180 - ((180/np.pi) * np.arctan(y/(-x)))
    elif (x < 0 and y < 0): 
        theta = 180 + ((180/np.pi) * np.arctan(y/x))
    elif ( x >= 0 and y < 0):
        theta = 360 - ((180/np.pi) * np.arctan((-y)/x))
    return theta


def pt2rs(point, gap_ring, gap_sector, num_ring, num_sector):
    x = point[0]
    y = point[1]
    # z = point[2]
    
    if(x == 0.0):
        x = 0.001
    if(y == 0.0):
        y = 0.001
    
    theta = xy2theta(x, y)
    faraway = np.sqrt(x*x + y*y)
    
    idx_ring = np.divmod(faraway, gap_ring)[0]       
    idx_sector = np.divmod(theta, gap_sector)[0]

    if(idx_ring >= num_ring):
        idx_ring = num_ring-1 # python starts with 0 and ends with N-1
    
    return int(idx_ring), int(idx_sector)


ENOUGH_LARGE = 500  # max points retained per (ring, sector) bin


def ptcloud2sc(ptcloud, sc_shape, max_length):
    """Vectorised equivalent of the original per-point loop.

    The reference implementation walked every point in Python, calling
    pt2rs()/xy2theta() with scalar numpy ops - ~330 ms for a KITTI keyframe
    cloud, which made this the second largest cost in the loop closure
    detector. This produces bit-identical output (verified against the
    original on random, KITTI-like, degenerate and bin-saturating inputs).

    pt2rs()/xy2theta() are kept below for reference and for any external
    caller, but are no longer on this path.
    """
    num_ring = sc_shape[0]
    num_sector = sc_shape[1]

    gap_ring = max_length/num_ring
    gap_sector = 360/num_sector
    nbins = num_ring * num_sector

    pts = np.asarray(ptcloud, dtype=np.float64)
    # Reject spurious points, as the original did per point
    pts = pts[~np.isnan(pts[:, :3]).any(axis=1)]
    if pts.shape[0] == 0:
        return np.zeros((num_ring, num_sector))

    x = pts[:, 0].copy()
    y = pts[:, 1].copy()
    x[x == 0.0] = 0.001
    y[y == 0.0] = 0.001

    # xy2theta's four explicit quadrant branches are exactly atan2 in [0, 360)
    theta = np.degrees(np.arctan2(y, x)) % 360.0
    faraway = np.sqrt(x * x + y * y)

    idx_ring = np.floor(faraway / gap_ring).astype(np.int64)
    idx_sector = np.floor(theta / gap_sector).astype(np.int64)
    np.clip(idx_ring, 0, num_ring - 1, out=idx_ring)
    np.clip(idx_sector, 0, num_sector - 1, out=idx_sector)

    heights = pts[:, 2] + 2.0  # for setting ground is roughly zero
    flat = idx_ring * num_sector + idx_sector

    # The original only stored the first ENOUGH_LARGE points to land in a bin,
    # in arrival order; a stable sort reproduces that ordering exactly.
    order = np.argsort(flat, kind='stable')
    flat_s = flat[order]
    h_s = heights[order]
    counts = np.bincount(flat_s, minlength=nbins)
    starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
    rank = np.arange(flat_s.shape[0]) - starts[flat_s]
    keep = rank < ENOUGH_LARGE

    sc = np.full(nbins, -np.inf)
    np.maximum.at(sc, flat_s[keep], h_s[keep])
    # sc_storage was zero-filled and np.amax ran over the unused slots too, so
    # any bin holding fewer than ENOUGH_LARGE points had an implicit 0 floor
    # (and an empty bin came out as exactly 0). Saturated bins had no unused
    # slot left, so they keep their true maximum.
    sc = np.where(counts < ENOUGH_LARGE, np.maximum(sc, 0.0), sc)

    return sc.reshape(num_ring, num_sector)


def sc2rk(sc):
    return np.mean(sc, axis=1)

def distance_sc(sc1, sc2):
    """Vectorised equivalent of the original doubly-nested column loop.

    The reference implementation compared every column pair for every one of
    num_sectors circular shifts - 3600 scalar np.dot/np.linalg.norm calls,
    ~92 ms per call. It is invoked once per candidate (10 per keyframe), so it
    dominated the loop closure detector at ~920 ms per keyframe.

    Rolling sc1 right by s makes its column j the original column (j-s) % N, so
    every shift reuses the same column-vs-column dot products. Computing them
    once as sc1.T @ sc2 turns the whole thing into one small matmul plus a
    gather. Results match the original to floating-point rounding (the
    summation order differs); the selected shift is identical.
    """
    num_sectors = sc1.shape[1]

    sc1 = np.asarray(sc1, dtype=np.float64)
    sc2 = np.asarray(sc2, dtype=np.float64)

    norm1 = np.linalg.norm(sc1, axis=0)
    norm2 = np.linalg.norm(sc2, axis=0)
    # A column is "engaged" iff it has any non-zero entry, which for a real
    # vector is exactly the condition that its norm is non-zero - the same test
    # the original spelled as ~np.any(col), and the same one that protected the
    # division below.
    valid1 = norm1 > 0.0
    valid2 = norm2 > 0.0

    # cos[a, b] = cosine similarity between sc1 column a and sc2 column b
    denom = np.outer(norm1, norm2)
    with np.errstate(invalid='ignore', divide='ignore'):
        cos = (sc1.T @ sc2) / denom
    engaged = np.outer(valid1, valid2)
    cos = np.where(engaged, cos, 0.0)

    # For shift s, column j of the rolled sc1 is original column (j - s) % N.
    j_idx = np.arange(num_sectors)[None, :]
    s_idx = np.arange(1, num_sectors + 1)[:, None]
    a_idx = (j_idx - s_idx) % num_sectors

    contrib = cos[a_idx, j_idx]
    counts = engaged[a_idx, j_idx].sum(axis=1)
    sums = contrib.sum(axis=1)
    sim_for_each_cols = np.where(counts > 0, sums / np.maximum(counts, 1), 0.0)

    yaw_diff = np.argmax(sim_for_each_cols) + 1 # because python starts with 0
    sim = np.max(sim_for_each_cols)
    dist = 1 - sim

    return dist, yaw_diff