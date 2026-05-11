# Robot Configuration, Degrees of Freedom, and Topology

## Configuration

The **configuration** of a robot is a complete specification of the positions of every point of the robot. For a rigid body in 3D this means position + orientation; for an articulated robot it is the set of joint values that pins down where every link is.

Examples:
- A door rotating about its hinge: one number — the hinge angle in $[0, 2\pi)$.
- A planar 2R arm: two joint angles $(\theta_1, \theta_2)$.
- A four-bar linkage: although it has four joints, three of them are determined once you pick one (the loop closure constraint), so the configuration is captured by **one** number.

## Configuration space (C-space)

The set of all configurations the robot can attain is its **configuration space**, written $\mathcal{Q}$ or $\mathcal{C}$. Geometrically it is a manifold; the dimension and shape (topology) depend on the robot.

## Degrees of freedom

The **degrees of freedom (dof)** is the minimum number of real numbers needed to specify a configuration. Equivalently, it is the dimension of the C-space.

For a system of rigid bodies connected by joints, dof is given by **Grübler's formula** (also called the Chebychev–Grübler–Kutzbach criterion):

$$
\boxed{\;\text{dof} \;=\; m\,(N - 1 - J) \;+\; \sum_{i=1}^{J} f_i\;}
$$

where
- $N$ = number of links **including the ground link**,
- $J$ = number of joints,
- $f_i$ = number of dof allowed by joint $i$,
- $m = 3$ for planar mechanisms, $m = 6$ for spatial mechanisms.

The formula counts the dof of $N - 1$ free moving bodies in the chosen space and subtracts the constraints imposed by each joint ($m - f_i$ per joint).

### Joint table

| Joint              | Symbol | dof ($f_i$) | Spatial constraints ($6 - f_i$) | Notes                                  |
|--------------------|--------|-------------|---------------------------------|----------------------------------------|
| Revolute (rotary)  | R      | 1           | 5                               | Rotation about a fixed axis            |
| Prismatic          | P      | 1           | 5                               | Translation along a fixed axis         |
| Helical (screw)    | H      | 1           | 5                               | Coupled rotation + translation         |
| Cylindrical        | C      | 2           | 4                               | Independent R + P about same axis      |
| Universal          | U      | 2           | 4                               | Two intersecting revolute axes         |
| Spherical          | S      | 3           | 3                               | Ball-and-socket                        |
| Planar             | E      | 3           | 3                               | 2 translations + 1 rotation in a plane |

### Worked examples

**4-bar linkage (planar):** $N = 4$, $J = 4$ revolute, $m = 3$:
$$\text{dof} = 3(4 - 1 - 4) + 4\cdot 1 = -3 + 4 = 1.$$

**Planar 5-bar linkage:** $N = 5$, $J = 5$ revolute:
$$\text{dof} = 3(5 - 1 - 5) + 5 = -3 + 5 = 2.$$

**Stewart–Gough platform (spatial):** 6 legs, each with one universal joint at the base ($f=2$), one prismatic ($f=1$), one spherical at the top ($f=3$). $N = 14$ (base + top platform + 6 lower legs + 6 upper legs), $J = 18$ (6 U + 6 P + 6 S):
$$\text{dof} = 6(14 - 1 - 18) + 6(2 + 1 + 3) = 6\cdot(-5) + 36 = 6.$$

**Delta robot (spatial):** 3 parallel arms, each arm is an R-(parallelogram of 4 S joints)-... Computing it carefully via Grübler gives **3 translational dof** for the end-effector (the parallelograms keep the moving platform's orientation fixed).

**General 6R industrial arm (e.g. UR5, Franka):** $N = 7$, $J = 6$, all R: $\text{dof} = 6(7 - 1 - 6) + 6 = 6$.

### When Grübler's formula fails

Grübler counts generic mobility. **Special geometric configurations** can have higher dof than the formula predicts:
- A planar four-bar with all four links the same length and aligned on a line (a parallelogram in a degenerate pose) momentarily gains a dof.
- A spatial mechanism with parallel or intersecting axes may have extra mobility (Bennett linkage: $\text{dof} = 1$ but Grübler predicts $-2$).

These are called **overconstrained** mechanisms and their analysis requires deeper kinematics (screw theory).

## Task space

The **task space** is the space in which the task is naturally expressed. It depends on the *task*, not the robot.

- Tracking a 2D pen tip on a whiteboard → task space is $\mathbb{R}^2$.
- Positioning + orienting a rigid tool in 3D (drilling, welding, pick-and-place) → task space is $SE(3)$, dimension 6.
- Controlling the height of a forklift → task space is $\mathbb{R}^1$.

A robot is **redundant** for a task if its dof exceeds the task-space dimension (e.g. a 7R arm performing a 6-dof pose task).

## Workspace

The **workspace** is the set of points (or poses) the end-effector can physically reach. It depends on the robot's geometry and joint limits, not on a particular task.

- **Reachable workspace:** the set of points the end-effector can reach with *at least one* orientation.
- **Dexterous workspace:** the set of points reachable with *all* orientations. Dexterous workspace ⊆ reachable workspace.

For a planar 2R arm with link lengths $\ell_1, \ell_2$ and unrestricted joints, the reachable workspace is an annulus
$$\{\,(x,y) : |\ell_1 - \ell_2| \le \sqrt{x^2 + y^2} \le \ell_1 + \ell_2\,\}.$$
With joint range limits, the workspace becomes a sub-region of this annulus.

## Topology of C-space

Two C-spaces of the same dimension can have very different *shapes*. Two spaces are **topologically equivalent** (homeomorphic) if one can be smoothly deformed into the other without cutting or gluing.

| System                 | dof | C-space topology              |
|------------------------|-----|-------------------------------|
| Point on a line        | 1   | $\mathbb{R}$                  |
| Point on a circle      | 1   | $S^1$ (circle)                |
| Point on a plane       | 2   | $\mathbb{R}^2$                |
| Spherical pendulum     | 2   | $S^2$ (sphere)                |
| Planar 2R arm          | 2   | $T^2 = S^1 \times S^1$ (torus)|
| Rigid body in plane    | 3   | $\mathbb{R}^2 \times S^1$     |
| Rigid body in 3D       | 6   | $SE(3) = \mathbb{R}^3 \times SO(3)$ |
| 3D rotations           | 3   | $SO(3)$ (real projective space $\mathbb{RP}^3$) |

The topology is intrinsic to the system — it does not depend on how we choose to *represent* it numerically.

## Representations of C-space

### Explicit (minimal) representation

Pick the smallest possible number of coordinates. For Euclidean spaces this is natural — choose an origin and orthogonal axes. For curved spaces it works locally but introduces **singularities**.

**Latitude/longitude on $S^2$:**
- Latitude: angle between the equatorial plane and the line from the center to the point. $+90°$ at the north pole, $-90°$ at the south pole.
- Longitude: angle east/west from a reference meridian.
- **Singularity at the poles:** longitude is undefined; a small physical motion across the pole produces a $180°$ jump in longitude.
- **Distortion:** moving at constant speed near a pole produces unbounded longitude rates while the same motion near the equator produces small ones.

Every minimal representation of $SO(3)$ (Euler angles, roll-pitch-yaw, axis-angle) has at least one such singularity — gimbal lock for ZYX Euler angles, for example.

### Implicit (constrained) representation

Use more coordinates than the dof, and impose algebraic constraints.

**Sphere:** represent a point on $S^2$ by $(x,y,z)$ subject to $x^2 + y^2 + z^2 = 1$. Three coordinates, one constraint, two effective dof. No singularities, but the derivative of the coordinates is *not* the velocity on the sphere — it is the ambient velocity, which must be projected onto the tangent plane.

**Rotations:** represent an element of $SO(3)$ by a $3\times 3$ rotation matrix $R$ subject to $R^{\top}R = I$ and $\det R = +1$. Nine numbers, six constraints, three effective dof. Singularity-free; the price is six redundant numbers and the need to enforce constraints under integration.

**Unit quaternions** are an in-between: four numbers with one constraint $\|q\| = 1$ → three dof, double-cover of $SO(3)$ ($q$ and $-q$ are the same rotation). Singularity-free, more compact than rotation matrices, easy to compose and renormalize.

### Configuration vs velocity constraints

A constraint on positions (like $x^2 + y^2 + z^2 = 1$) is a **holonomic** constraint and reduces the dof of the C-space by one per independent equation.

A constraint that involves velocities and cannot be integrated to a position constraint (like the rolling-without-slipping condition of a unicycle) is **nonholonomic** — it does *not* reduce the C-space dimension, but it does restrict the directions in which the system can move at each configuration. A unicycle has a 3-dim C-space ($x, y, \theta$) but only 2 controllable velocity dof at any instant.

## Why this matters

Choosing the right representation is a tradeoff between:
- **Minimality:** fewer numbers to track, but you accept singularities.
- **Singularity-free behavior:** redundant numbers + constraints, but the coordinate derivative is no longer the velocity.

For robot dynamics, planning, and control:
- Joint-space representations (one number per joint) are usually minimal and convenient for serial arms.
- For free-flying rigid bodies, $SE(3)$ with quaternions or rotation matrices avoids singularities and is the standard choice in modern robotics libraries (Eigen, Sophus, manif).

---

Refs:
- Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control* — [book site](http://hades.mech.northwestern.edu/index.php/Modern_Robotics), [code repo](https://github.com/NxRLab/ModernRobotics).
- Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*.
- Featherstone, *Rigid Body Dynamics Algorithms* (for the joint type taxonomy in modern simulators).
