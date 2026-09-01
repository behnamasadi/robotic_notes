# Information Form vs Covariance Form

**Which parameterization of a Gaussian should an estimator carry — $(\mu,\Sigma)$ or $(\eta,\Lambda)$ — and why?**

The short answer: they are two coordinate systems on the same object, and the operations
an estimator needs are *linear in different ones*. Fusion and conditioning are addition in
information coordinates; prediction and marginalization are addition-like in moment
coordinates. Every design in this repo — Kalman filter, information filter, pose-graph
backend, factor graph, fixed-lag smoother — is a choice about which asymmetry to pay for.

This note collects the argument in one place. The pieces live in
[nonlinear_least_squares.ipynb](nonlinear_least_squares.ipynb) (where $\Omega$ comes from),
[kalman_filter.ipynb](kalman_filter.ipynb) (the gain),
[factor_graph.ipynb](factor_graph.ipynb) (elimination and marginalization) and
[pose_graph_slam.ipynb](pose_graph_slam.ipynb) (the assembled $H$); this is the comparison
that none of them makes head-on.

- [1. Two coordinate systems on the same object](#1-two-coordinate-systems-on-the-same-object)
- [2. The underlying operation is multiplication](#2-the-underlying-operation-is-multiplication)
- [3. The Kalman gain is the price of moment coordinates](#3-the-kalman-gain-is-the-price-of-moment-coordinates)
- [4. Then why does anyone use the covariance form?](#4-then-why-does-anyone-use-the-covariance-form)
- [5. Why SLAM back-ends live in information space](#5-why-slam-back-ends-live-in-information-space)
- [6. Why $H$ is literally an information matrix](#6-why-h-is-literally-an-information-matrix)
- [7. Marginalization: the bill information form eventually pays](#7-marginalization-the-bill-information-form-eventually-pays)
- [8. Square-root forms: refuse to form either matrix](#8-square-root-forms-refuse-to-form-either-matrix)
- [9. Decision table](#9-decision-table)
- [10. Worked numbers](#10-worked-numbers)

---

## 1. Two coordinate systems on the same object

A Gaussian can be written with **moment parameters** $(\mu, \Sigma)$:

$$
p(x) \;\propto\; \exp\!\Big(-\tfrac12 (x-\mu)^\top \Sigma^{-1} (x-\mu)\Big)
$$

or with **natural** (canonical) **parameters** $(\eta, \Lambda)$:

$$
p(x) \;\propto\; \exp\!\Big(-\tfrac12 x^\top \Lambda x + x^\top \eta\Big),
\qquad
\Lambda = \Sigma^{-1},\quad \eta = \Sigma^{-1}\mu
$$

Same distribution, different chart. The map between them is nonlinear — it contains a
matrix inversion — and that single fact generates every asymmetry below.

Two consequences that are easy to miss:

* **$\Lambda = 0$ is a legal state; $\Sigma = \infty$ is not.** "I know nothing yet" is the
  origin in information coordinates, and it is exactly what `H = np.zeros(...)` means at
  the top of every Gauss-Newton iteration
  ([nonlinear_least_squares §3.1](nonlinear_least_squares.ipynb#31-initialize-linear-system)).
  Moment coordinates cannot represent it: there is no $\Sigma$ whose inverse is zero.
* **$\Lambda$ can be singular and still be meaningful.** A rank-deficient $\Lambda$ says
  "some directions are unconstrained" — the gauge freedom of a pose graph is exactly this
  (§5). The corresponding $\Sigma$ does not exist.

---

## 2. The underlying operation is multiplication

Fusing two independent Gaussian beliefs about the same quantity means multiplying
densities. In log space:

$$
-2\log p \;=\; (x-\mu_1)^\top\Lambda_1(x-\mu_1) + (x-\mu_2)^\top\Lambda_2(x-\mu_2) + \text{const}
$$

Expand: the quadratic terms give $\Lambda_1+\Lambda_2$, the linear terms give
$\Lambda_1\mu_1+\Lambda_2\mu_2$. So in natural parameters,

$$
\boxed{\;\Lambda = \Lambda_1+\Lambda_2, \qquad \eta = \eta_1+\eta_2\;}
$$

That is the whole operation. Gaussians are an exponential family, and a product of
exponential-family densities is a **sum of natural parameters** — addition is the *native*
operation in this chart. It is associative, commutative, and order-independent, which is
why you can accumulate measurements in any order and stop whenever you like.

---

## 3. The Kalman gain is the price of moment coordinates

Insist on staying in moment coordinates and you must push that trivial addition through a
matrix inversion:

$$
\Sigma = \big(\Sigma_1^{-1}+\Sigma_2^{-1}\big)^{-1},
\qquad
\mu = \Sigma\big(\Sigma_1^{-1}\mu_1+\Sigma_2^{-1}\mu_2\big)
$$

Rewrite the mean as an increment on $\mu_1$:

$$
\mu = \Sigma\big((\Lambda_1+\Lambda_2)\mu_1 + \Lambda_2(\mu_2-\mu_1)\big)
    = \mu_1 + \underbrace{\Sigma\Lambda_2}_{\textstyle K}(\mu_2-\mu_1)
$$

and by the matrix inversion lemma,

$$
K \;=\; \Sigma\Lambda_2 \;=\; \big(\Sigma_1^{-1}+\Sigma_2^{-1}\big)^{-1}\Sigma_2^{-1}
      \;=\; \Sigma_1\big(\Sigma_1+\Sigma_2\big)^{-1}
$$

With a measurement model $z = Hx + v$, $v\sim\mathcal N(0,R)$, the same algebra gives the
familiar $K = \Sigma H^\top(H\Sigma H^\top + R)^{-1}$.

**So the gain is not extra physics.** It is the correction you pay for expressing an
addition in inverted coordinates — the Jacobian of the chart change, in effect. The scalar
case makes it plain:

$$
\mu \;=\; \frac{\mu_1/\sigma_1^2+\mu_2/\sigma_2^2}{1/\sigma_1^2+1/\sigma_2^2}
$$

which is inverse-variance weighting: add $\eta$, add $\Lambda$, divide. The
[Kalman-filter notebook](kalman_filter.ipynb#where-the-kalman-gain-comes-from) derives $K$
by completing the square; this is the same result read off the other chart, where no
completing of squares is necessary because nothing needed rearranging.

---

## 4. Then why does anyone use the covariance form?

Because the *other* half of a filter flips the asymmetry. A recursive estimator does two
things, and each is easy in a different chart.

| operation | cheap in **information** $(\eta,\Lambda)$ | cheap in **moment** $(\mu,\Sigma)$ |
|---|---|---|
| **fuse / update** (condition on a measurement) | $\Lambda^+ = \Lambda^- + H^\top R^{-1}H$, $\eta^+ = \eta^- + H^\top R^{-1}z$ — addition | needs the gain: form $S = H\Sigma H^\top + R$ ($m\times m$), invert it |
| **predict** (propagate through $x^+ = Fx + w$) | $\Lambda^+ = \big(F\Lambda^{-1}F^\top + Q\big)^{-1}$ — two $n\times n$ inversions | $\Sigma^+ = F\Sigma F^\top + Q$ — no inversion |
| **marginalize** (drop a variable) | Schur complement, and it creates fill-in | delete the rows and columns |
| **condition** (fix a variable) | delete the rows and columns | Schur complement |

Read the table diagonally: **conditioning is trivial in $\Lambda$, marginalization is
trivial in $\Sigma$**, and they are exactly each other's dual. The Kalman filter picks
moment coordinates and pays at the update (the gain, an $m\times m$ inverse); the
information filter picks natural coordinates and pays at the prediction (an $n\times n$
inverse every step, even when nothing was measured).

**The dimension that decides it.** In the update, $m$ is the measurement dimension and $n$
the state dimension.

* $m \gg n$ — thousands of landmark observations against a small state. The KF inverts an
  $m\times m$ innovation covariance; the information filter adds
  $H^\top R^{-1} H$ and never forms it. Information wins, decisively.
* $n \gg m$ — a large map, a handful of measurements per step. The prediction inversion
  dominates and moment form wins.

One honest caveat on "the information update needs no inversion": it needs $R^{-1}$. That
is free when $R$ is diagonal or block-diagonal — independent sensors, independent
landmark observations — which is the case that matters, because then each measurement
contributes its own small block and they simply add. Correlated $R$ across a huge
measurement vector removes the advantage.

---

## 5. Why SLAM back-ends live in information space

Batch SLAM is all update and no prediction, which is precisely the regime where
information form wins. Concretely, in
[pose_graph_slam §8.3](pose_graph_slam.ipynb#83-global-normal-equations-how-each-edge-adds-into-h-and-b)
and [nonlinear_least_squares §3.6](nonlinear_least_squares.ipynb#36-update-information-matrix-h):

$$
H = \sum_{(i,j)} J_{ij}^\top \Omega_{ij} J_{ij},
\qquad
b = \sum_{(i,j)} J_{ij}^\top \Omega_{ij} e_{ij}
$$

The `+=` in that loop *is* the information-form additivity of §2. Independent measurements
add information: no gain, no inversion, order-independent, and the single
`solve(H, b)` at the end is the only place the estimator converts back to moment space.
Four properties follow, and each one is information-form behaviour rather than a
coincidence of least squares:

**Sparsity is conditional independence.** $H_{ij} \ne 0$ iff some factor couples $i$ and
$j$ directly. That is the Gaussian Markov random field property of $\Lambda$: a zero
off-diagonal block means $x_i \perp x_j$ *given the rest*. In moment form the same belief
has a dense $\Sigma$ — everything is correlated with everything through the chain. The
graph is sparse in exactly one of the two charts, and back-ends are built in that one.
([factor_graph §3](factor_graph.ipynb#3-information-form-covariance-sigma-vs-information-lambda))

**Starting from zero.** `H = 0` is "no information yet", a state moment form cannot hold
(§1).

**Gauge fixing is injecting information.** $H$ from relative measurements alone is
singular, because relative measurements carry *zero information* about the global frame —
the null space is the gauge directions. Anchoring $X_0$ adds an information block to
$H_{00}$, whether spelled `H[0:3,0:3] += 1e6*I`
([nonlinear_least_squares §3.7](nonlinear_least_squares.ipynb#37-apply-gauge-fixing)) or as
a genuine prior factor ([factor_graph §4](factor_graph.ipynb#4-a-running-example-5-poses-with-one-loop-closure)).
Those are the same act: supplying the missing information.

**LM damping is a prior.** $H + \lambda I$ adds isotropic information $\lambda$ centred at
the current estimate — a soft trust region. Large $\lambda$ = strong prior = short step,
tending to gradient descent
([nonlinear_least_squares §3.11](nonlinear_least_squares.ipynb#311-optional-levenbergmarquardt-damping)).

---

## 6. Why $H$ is literally an information matrix

Calling $H$ "the information matrix" is not loose terminology. Three independent routes
arrive at the same object.

**(a) It is the Hessian of the negative log posterior.** The cost
$E(x)=\tfrac12\sum e_{ij}^\top\Omega_{ij}e_{ij}$ *is* $-\log p(z\mid x)$ up to a constant
for Gaussian noise. Linearize $e(x+\delta)\approx \bar e + J\delta$; then
$\partial^2 E/\partial\delta^2 = J^\top\Omega J = H$. A Gaussian's exponent is
$\tfrac12\delta^\top\Lambda\delta$, so the Hessian of its negative log density *is*
$\Lambda$. Same object, reached from the optimization side.

**(b) It is the Fisher information, exactly.** For $z = h(x)+v$ with
$v\sim\mathcal N(0,\Omega^{-1})$,

$$
\mathcal I(x) \;=\; \mathbb E\big[\nabla_x\log p\;\nabla_x\log p^\top\big] \;=\; J^\top\Omega J
$$

$\Omega$ is information in *measurement* space; $J$ maps a state perturbation to a
measurement perturbation; $J^\top\Omega J$ pulls that information back into *state* space.
This identity is exact for the Gaussian model — it does not depend on Gauss-Newton being a
good Hessian approximation.

**(c) Laplace closes the loop.** At the optimum, the posterior is approximately
$\mathcal N(x^\star, H^{-1})$, so $\Sigma_{\text{post}} = H^{-1}$ — which is exactly the
inverse-Hessian recipe for obtaining $\Sigma$ for the next graph up the stack
([nonlinear_least_squares §1.5.1](nonlinear_least_squares.ipynb#151-the-inverse-hessian-recipe-and-why-its-the-same-formula-everywhere)).
A scan matcher's $H$ becomes an edge's $\Omega$ in the pose graph; the pose graph's $H$
becomes the prior for whatever consumes it. The whole hierarchy is information matrices
feeding information matrices, and the inversion happens only at the interfaces.

### The caveat

$H = J^\top\Omega J$ is the **Gauss-Newton** Hessian. The true one is

$$
\nabla^2 E \;=\; J^\top\Omega J \;+\; \sum_k (\Omega e)_k \,\nabla^2 e_k
$$

The second term is dropped. It vanishes at zero residual and for linear $h$, and is
usually small near convergence — which is why Gauss-Newton works
([pose_graph_slam §7.5.2](pose_graph_slam.ipynb#752-solving-for-the-optimal-delta-x-differentiate-and-set-to-zero)).
So: $H$ is *exactly* the Fisher information, and the *approximate* Hessian.

With a robust kernel (Huber, Cauchy), $\Omega$ is replaced by a reweighted
$w(e)\,\Omega$, and $H$ is then the information of the *equivalent reweighted Gaussian
problem*, not of the original one. Covariances read off a robustified $H$ are
correspondingly optimistic about the down-weighted measurements.

---

## 7. Marginalization: the bill information form eventually pays

Everything above says "use information form". Marginalization is where that stops being
free, and it is the reason production systems are not purely one or the other.

Dropping variable $b$ from a joint belief over $(a,b)$:

$$
\text{moment form: } \Sigma_{aa}\ \text{(delete rows/cols)},
\qquad
\text{information form: } \Lambda_{aa} - \Lambda_{ab}\Lambda_{bb}^{-1}\Lambda_{ba}
$$

The Schur complement is not merely more arithmetic — it **destroys sparsity**. Marginalizing
a variable connects all of its former neighbours to each other, because information that
used to flow *through* it now has to flow directly. On a chain, marginalizing one interior
node turns two edges into one new edge; do it repeatedly, on a node with many neighbours,
and the once-sparse $\Lambda$ fills in until the sparse solve is no longer sparse.

This is the whole tension in fixed-lag smoothing: bounded memory requires marginalizing old
poses, and marginalizing old poses is what makes the remaining matrix dense
([factor_graph §10](factor_graph.ipynb#10-fixed-lag-smoothing--bounded-memory-with-a-catch)).
It is also the same operation as variable elimination
([factor_graph §5](factor_graph.ipynb#5-marginalization-and-the-schur-complement)) — which
is why elimination *ordering* matters: it is a fill-in minimization problem.

And it explains the standard hybrid. Keep the batch problem in information form, and use
the moment form only where it is genuinely needed — reporting per-variable covariance to a
consumer, or gating a measurement — recovering just the blocks of $\Sigma$ you actually
need rather than inverting $\Lambda$ wholesale.

---

## 8. Square-root forms: refuse to form either matrix

Both $\Lambda$ and $\Sigma$ have condition number $\kappa$; their square-root factors have
$\sqrt{\kappa}$. Forming $H = J^\top\Omega J$ explicitly squares the conditioning of $J$ —
information that was representable in $J$ can be lost in $H$.

The square-root variants avoid ever forming the product:

* **SRIF / QR back-ends** keep the triangular $R$ with $\Lambda = R^\top R$, obtained by QR
  on the whitened Jacobian directly. This is what iSAM2 stores, and why
  [factor_graph §6.6](factor_graph.ipynb#66-square-root-form-from-lambda-to-r-via-whitening-and-qr)
  spends a section on it.
* **Square-root Kalman filters** keep a factor of $\Sigma$ and update it in place.

Same belief, a third chart, chosen for numerical rather than structural reasons.

---

## 9. Decision table

| your situation | use | because |
|---|---|---|
| Batch MAP over a whole trajectory (pose graph, bundle adjustment, factor graph) | **information** | all update, no prediction; measurements add; sparsity = graph structure |
| Recursive filter, small state, frequent prediction (attitude, single-body EKF) | **moment** | prediction is the common operation and is inversion-free |
| Many measurements per step, $m \gg n$ (dense landmarks, multi-sensor fusion) | **information** | never forms the $m\times m$ innovation covariance |
| Large state, few measurements, $n \gg m$ | **moment** | avoids an $n\times n$ inversion at every prediction |
| Need per-variable uncertainty as output | **moment**, at the boundary only | consumers want $\Sigma$; recover blocks, don't invert wholesale |
| Must drop old variables to bound memory | **moment** for the drop, and accept fill-in in $\Lambda$ | marginalization is deletion in $\Sigma$, Schur in $\Lambda$ |
| No prior information at all to start from | **information** | $\Lambda = 0$ exists; $\Sigma = \infty$ does not |
| Ill-conditioned $J$, long time horizons | **square root** of either | $\sqrt{\kappa}$ instead of $\kappa$ |

> **Rule of thumb.** Ask which operation you perform most. Fusing and conditioning →
> information. Predicting and marginalizing → moment. Doing both a lot → keep information
> internally and convert at the interfaces, which is what every SLAM stack in this repo does.

---

## 10. Worked numbers

Each of these is small enough to check by hand, and all were verified numerically.

**Scalar fusion.** $\mu_1=2.0,\ \sigma_1=1.0$ and $\mu_2=3.0,\ \sigma_2=0.5$:

$$
\Lambda = 1 + 4 = 5,\qquad
\eta = (1)(2) + (4)(3) = 14,\qquad
\mu = \tfrac{14}{5} = 2.8,\qquad \sigma^2 = \tfrac15 = 0.2
$$

Via the gain instead: $K = \sigma_1^2/(\sigma_1^2+\sigma_2^2) = 1/1.25 = 0.8$, so
$\mu = 2 + 0.8(3-2) = 2.8$ and $\sigma^2 = (1-K)\sigma_1^2 = 0.2$. Identical, as it must be
— the second is the first pushed through the chart change.

**The two gain formulas agree.** With

$$
\Sigma_1=\begin{pmatrix}2.0&0.3\\0.3&1.0\end{pmatrix},
\qquad
\Sigma_2=\begin{pmatrix}0.5&-0.2\\-0.2&0.8\end{pmatrix}:
$$


$$
\Sigma\Lambda_2 \;=\; \Sigma_1(\Sigma_1+\Sigma_2)^{-1}
\;=\;\begin{pmatrix} 0.7951 & 0.1225\\ 0.0980 & 0.5501\end{pmatrix}
$$

and both routes to the fused mean give $(1.2751,\ 1.4989)$.

**KF update $\equiv$ IF update.** For a random $n=4$ state and $m=7$ measurement, the
moment-form update $\big(K = PH^\top S^{-1}$, $x^+ = x + K(z-Hx)\big)$ and the
information-form update $\big(\Lambda^+ = \Lambda + H^\top R^{-1}H$,
$\eta^+ = \eta + H^\top R^{-1}z\big)$ agree to $7\times10^{-16}$ in the mean and
$1\times10^{-15}$ in the covariance — floating-point noise. They are the same update.

**Marginalization fills in.** Take a 5-node chain whose $\Lambda$ is tridiagonal, and
marginalize the middle node. In $\Sigma$: delete a row and column. In $\Lambda$: a Schur
complement, after which nodes 1 and 3 — previously unconnected, $\Lambda_{13}=0$ exactly —
acquire a coupling of $-0.144$. One elimination, one new edge; that is fill-in, and it is
the entire cost story of §7.

For the same thing on a real pose graph rather than a toy chain, the runnable cell in
[factor_graph §5](factor_graph.ipynb#5-marginalization-and-the-schur-complement) marginalizes
a pose out of the 5-pose example: $\Lambda_{13}$ goes from exactly zero to a block carrying
$\approx 25$ per translation axis and $\approx 50$ on $\theta$ — half of each edge's
information, because two odometry constraints in series compose like resistors — and it
confirms that inverting the Schur complement reproduces the corresponding sub-block of
$\Sigma$ to $2\times10^{-17}$.

---

## See also

- [nonlinear_least_squares.ipynb](nonlinear_least_squares.ipynb) — where $\Omega$ comes from, and the normal equations
- [kalman_filter.ipynb](kalman_filter.ipynb) — the gain derived by completing the square
- [factor_graph.ipynb](factor_graph.ipynb) — elimination, Schur complements, square-root form, fixed-lag smoothing
- [pose_graph_slam.ipynb](pose_graph_slam.ipynb) — the assembled $H$, its sparsity and its gauge
- [map.ipynb](map.ipynb) — MAP as weighted least squares
- [probability_review.ipynb](probability_review.ipynb) — Gaussian identities
