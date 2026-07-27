# Rotation Averaging

Notes on averaging rotations, and on the role rotation averaging plays in global
Structure-from-Motion.

Two distinct problems share this name. Separating them first, because the answer
is different for each.

| | **Single rotation averaging** | **Multiple rotation averaging** |
|---|---|---|
| also called | rotation mean, attitude averaging | rotation synchronization |
| input | $N$ estimates of the *same* rotation | relative rotations $R_{ij}$ on a view graph |
| output | one rotation | one absolute rotation per camera |
| closed form? | yes (chordal), no (geodesic) | no — graph optimization |
| where it shows up | sensor fusion, RANSAC consensus | global SfM, pose-graph SLAM |

---

## 1. Single rotation averaging

You have $N$ estimates of the same rotation and want their average.

The naive elementwise mean fails immediately: the average of rotation matrices is
not a rotation matrix. It is not orthogonal, and it shrinks toward zero as the
inputs spread apart. So every method is really *"average in some ambient space,
then project back onto $SO(3)$"*.

### Chordal / Frobenius mean — closed form

Minimize the sum of squared Frobenius distances:

$$\bar{R} \;=\; \arg\min_{R \in SO(3)} \; \sum_i \lVert R - R_i \rVert_F^2$$

Solution — accumulate, then SVD:

$$A = \sum_i R_i, \qquad A = U \Sigma V^\top, \qquad
\bar{R} = U \, \operatorname{diag}\!\big(1,\, 1,\, \det(U V^\top)\big) \, V^\top$$

The $\det$ term is what keeps you in $SO(3)$ rather than landing on a reflection
($\det = -1$). Do not skip it.

This is the same computation as **Wahba's problem**, **Kabsch alignment**, and the
**orthogonal Procrustes problem** — all three are derived and worked through in
[shape_analysis.ipynb](shape_analysis.ipynb). You have probably implemented it
before under one of those names.

```python
import numpy as np

def chordal_mean(rotations, weights=None):
    """L2 mean of a list of 3x3 rotation matrices, closed form."""
    R = np.asarray(rotations)                      # (N, 3, 3)
    w = np.ones(len(R)) if weights is None else np.asarray(weights)
    A = np.einsum('n,nij->ij', w, R)
    U, _, Vt = np.linalg.svd(A)
    d = np.sign(np.linalg.det(U @ Vt))             # guard against reflection
    return U @ np.diag([1.0, 1.0, d]) @ Vt
```

### Quaternion mean

Build the $4 \times 4$ matrix $M = \sum_i w_i \, q_i q_i^\top$ and take the
eigenvector belonging to the largest eigenvalue. This is Markley's method, and it
is equivalent to the chordal mean.

**Gotcha:** quaternions double-cover $SO(3)$, so $q$ and $-q$ are the same
rotation. Hemisphere-align the inputs (flip any $q_i$ with $q_i \cdot q_0 < 0$) or
they will cancel each other out.

### Karcher / geodesic / Fréchet mean — iterative

Minimize squared *geodesic* (angular) distance instead of chordal. No known closed
form; iterate in the tangent space:

$$R \;\leftarrow\; R \, \exp\!\left( \frac{1}{N} \sum_i \log\big(R^\top R_i\big) \right)$$

This is the intrinsic mean on the manifold ($\exp$ and $\log$ here are the $SO(3)$
maps — see [lie_group_lie_algebra.ipynb](lie_group_lie_algebra.ipynb)). It differs
from the chordal mean when the rotations are widely spread; for tightly clustered
rotations the difference is negligible and the SVD version is far cheaper. Manton
gives a convergent algorithm.

### Robust variants

L1 / geodesic median computed with the **Weiszfeld algorithm** on $SO(3)$.
Tolerates outliers that both L2 means above do not. Relevant whenever the
estimates come from something that can fail outright, rather than just being
noisy. (The same Weiszfeld iteration shows up in DUSt3R's focal-length estimator —
see [dust3r_mast3r.ipynb](visual_odometry/dust3r_mast3r.ipynb).)

---

## 2. Multiple rotation averaging — the SfM problem

This is the one in global SfM pipelines. Given relative rotations $R_{ij}$ from
two-view geometry (essential matrix decomposition) over a view graph, recover
global absolute rotations $R_i$ such that:

$$R_{ij} \approx R_j R_i^\top$$

as a minimization over the edges $E$ of the view graph:

$$\min_{R_1 \ldots R_N} \; \sum_{(i,j) \in E} d\big(R_{ij},\, R_j R_i^\top\big)^p$$

### Why bother

Rotations decouple from translations. Solving all rotations globally *first* means
the pipeline becomes:

```
two-view geometry → rotation averaging → translation averaging → one bundle adjustment
```

instead of the incremental grow-and-re-BA loop that COLMAP uses, which drifts and
scales badly.

### The lineage

- **Govindu, CVPR 2001 / 2004** — first framed motion averaging this way; the 2004
  paper does the Lie-algebraic iteration in $\mathfrak{so}(3)$.
- **Martinec & Pajdla, CVPR 2007** — drop the orthonormality constraint, solve a
  linear least squares, project each block back to $SO(3)$. Still the standard
  cheap initialization.
- **Chatterjee & Govindu, ICCV 2013** — *Efficient and Robust Large-Scale Rotation
  Averaging*. L1 solution as initialization, then IRLS with a robust loss. The
  workhorse: if a pipeline does rotation averaging, it is probably running some
  descendant of this. Extended in *Robust Relative Rotation Averaging*, PAMI 2017.
- **Shonan Rotation Averaging**, Dellaert, Rosen, Wu, Mahony & Carlone, ECCV 2020 —
  convex relaxation lifting to $SO(p)$ for increasing $p$, giving *certifiable*
  global optimality rather than a local minimum. Implemented in GTSAM.

### The survey to actually read

**Hartley, Trumpf, Dai & Li, "Rotation Averaging," IJCV 103(3):267–305, 2013.**

Covers both problems, all three metrics (angular, chordal, quaternionic), the
L1/L2/L∞ variants, and the convergence guarantees. If you read one thing, read
this.

---

## Current state

- **GLOMAP** — *Global Structure-from-Motion Revisited*, ECCV 2024. A global
  pipeline matching or beating COLMAP's accuracy while being orders of magnitude
  faster, which had not previously been true.
- **ACD** — anisotropic coordinate descent, BMVC 2025. Speed and robustness.
- **Gravity-aligned rotation averaging** with circular regression, ECCV 2024 —
  exploits an IMU when you have one.
- **NeuRoRA**, ECCV 2020 — learned robust rotation averaging.

## The practical note

In SfM the hard part is almost never the averaging itself — it is **outlier
relative rotations** from bad two-view matches.

That is why the robust / IRLS line of work dominates over the elegant closed-form
solutions, and why **cycle-consistency filtering on the view graph before
averaging** is usually worth more than a better optimizer.

---

## Related notes in this repo

- [shape_analysis.ipynb](shape_analysis.ipynb) — Procrustes, Wahba, Kabsch,
  QUEST, Umeyama: the closed-form machinery §1 is built on.
- [lie_group_lie_algebra.ipynb](lie_group_lie_algebra.ipynb) — $\exp$/$\log$ on
  $SO(3)$, used by the Karcher mean and by the Lie-algebraic solvers of §2.
- [pose_graph_slam.ipynb](pose_graph_slam.ipynb) and
  [factor_graph.ipynb](factor_graph.ipynb) — the same synchronization problem on
  $SE(3)$, solved jointly with translations instead of rotations first.
- [sfm.ipynb](sfm.ipynb) — where the relative rotations $R_{ij}$ come from.
- [g2o.md](g2o.md) — a solver you can throw the graph of §2 at.

---

## References

- [Hartley, Trumpf, Dai & Li — *Rotation Averaging*, IJCV 2013](https://users.cecs.anu.edu.au/~hartley/Papers/PDF/Hartley-Trumpf:Rotation-averaging:IJCV.pdf)
- [Markley et al. — *Averaging Quaternions*](http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf)
- [Chatterjee & Govindu — *Efficient and Robust Large-Scale Rotation Averaging*, ICCV 2013](https://openaccess.thecvf.com/content_iccv_2013/html/Chatterjee_Efficient_and_Robust_2013_ICCV_paper.html)
- [Chatterjee & Govindu — *Robust Relative Rotation Averaging*, PAMI 2017](https://pubmed.ncbi.nlm.nih.gov/28422652/)
- [IISc CVLab — rotation averaging code and project page](https://ee.iisc.ac.in/cvlab/research/rotaveraging/)
- [Pan & Baráth et al. — *Global Structure-from-Motion Revisited* (GLOMAP), ECCV 2024](https://link.springer.com/chapter/10.1007/978-3-031-73661-2_4)
- [Lochman et al. — *ACD: Making Rotation Averaging Fast and Robust with Anisotropic Coordinate Descent*, BMVC 2025](https://ylochman.github.io/acd)
- [*Gravity-aligned Rotation Averaging with Circular Regression*, ECCV 2024](https://link.springer.com/chapter/10.1007/978-3-031-73661-2_6)
- [Purkait et al. — *NeuRoRA: Neural Robust Rotation Averaging*, ECCV 2020](https://link.springer.com/chapter/10.1007/978-3-030-58586-0_9)
