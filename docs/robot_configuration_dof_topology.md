# Configuration of Robot
The configuration of something answers the question, where is that thing? position of all point of a robot. For example, to know where a door is, we only need to know the angle about its hinge when it changes from 0 to 360 degrees, or for a four linkage robot if we have only one joint value we know the others so we only need 1 of them to find out where is the robot

The configuration of robot is a representation of the positions of all the points of the robot.

# Configuration  Space - (C-space )
the space of all configurations of the robot is called the configuration space or C-space of the robot


# Degrees of freedom.
The minimum number of real numbers that are needed for our representation is called the degrees of freedom.

# Task Space 
is a space in which the robot's tasks can be naturally expressed.
i.e. If the task is to control the position of a marker on board then the task space is euclidean plane.
or if the task is control the position and orientation of rigid body then the task space is 6dim space of rigid body
you only have to know about the task not the robot to define the task space


# Work Space
The Cartesian points that eef can reach and has nothing to do with a particular task. for instance a planar robot
with 2 revolute joint limited to range of motion 90 and 45 degree 

# Dexterous space:
The set of positions that can be reached with all possible orientations is called dexterous space


# dof
dof=sum (freedom of bodies ) - number of independent  constraint
constraint are often coming from joint


- prismatic joint 1
- rotary joint 1
- revolute joint 1
- Cylindrical joint 2
- helical joint 1
- universal joint 2
- spherical joint 3


grubler formula
delta robot
Stewart mechanism





# Topology
In addition to dof an other important property of C-space is Topology (or its shape), 
surface of sphere and surface or a plane. This difference in shape impact the way we use coordinate to represent the space.
Two space has the same shape or topologically equivalent if one can be smoothly deformed to the other without cutting and gluing

By definition, we call two spaces to be topologically equivalent or of the same shape if we can smoothly deform one to the other without cutting or gluing.

 

C-space of same dimension can have different topology, examples:

- Point on a plane -> Plane
- C-space of spherical pendulum ->  Sphere
- 2R robot -> torus


Topology of C-Space is independent of representation of space!


## Different way to represent C-space

### Explicit
1) Explicit: Points on a plane (or generally N-Dimensional Euclidean Space)
If the space is flat like a line or plane o generally n dimensional euclidean  space  we typically choose origin and coordinate axis and then use coordinate to represent the point
velocity is time derivative of those points


	choose 1 arbitrary point in the space and two orthogonal axis

### Implicit
2) Implicit: If the space is curved like sphere we can use explicit or implicit representation. (surface of a sphere for instance )

I ) Explicit particularization with min number of coordinate: latitude, longitude
problems: representation have poor behavior at some points, for instance if you travel with constant speed around equator your equator/ poles
your longitude change very slow/ rapid and no upper bound as you close to poles
north pole is the singularity of representation
Also the the moment you step over north pole your longitude will change by 180 degree


Latitude: angle between the equatorial plane and the straight line that passes through that point and the center of the Earth
The North Pole is 90° N; the South Pole is 90° S

longitude: angle east or west of a reference meridian to another meridian that passes through that point

North pole is called the singularity of the representation.

II ) Implicit subject to constraint x^2 + y^2 + z^2 = 1


The singularity free implicit representation we use is called rotation matrix. The derivative is not velocity

configuration constrain and velocity constraint


  

The representation of c-space doesn't change the underlying space itself therefor the topology of the space is independent of its representation of it space.


Refs: [1](http://hades.mech.northwestern.edu/index.php/Modern_Robotics), [2](https://github.com/NxRLab/ModernRobotics)


# Important Lie Groups and Typologies

<br/>

<img src="images/topology_example.png" alt="images/topology_example.png" />

<br/>
<br/>


## so(3)
The set of all 3x3 skew-symmetric real matrices is called <img src="https://latex.codecogs.com/svg.latex?so%283%29" alt="https://latex.codecogs.com/svg.latex?so(3)" />

Rotation Matrix: The space of orientation of a rigid body has only 3 dimension but rotation matrix is 3x3 but that mean the 9 entries in matrix subject to 6 constraint. These constraint are:
- 3 column vector are unit vector 
- They are two by two orthogonal

These constraint can written as <img src="https://latex.codecogs.com/svg.latex?R%5ETR%3DI" alt="https://latex.codecogs.com/svg.latex?R^TR=I" /> which ensure <img src="https://latex.codecogs.com/svg.latex?det%28R%29%3D1" alt="https://latex.codecogs.com/svg.latex?det(R)=1" />

## SO(2)
The group of rotations in two dimensions. The set of all 2×2 proper orthogonal matrices. They have the structure: 

<img src="https://latex.codecogs.com/svg.latex?R%3D%5Cbegin%7Bbmatrix%7D%20cos%28%5Ctheta%29%20%26%20sin%28%5Ctheta%29%20%5C%5C%20sin%28%5Ctheta%29%20%26%20-cos%28%5Ctheta%29%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?R=\begin{bmatrix}
cos(\theta) & sin(\theta) \\ 
sin(\theta) & -cos(\theta)
\end{bmatrix}" />






## SO(3)

<img src="https://latex.codecogs.com/svg.image?SO(3)" title="https://latex.codecogs.com/svg.image?SO(3)" />: special orthogonal group <img src="https://latex.codecogs.com/svg.image?SO(3)" title="https://latex.codecogs.com/svg.image?SO(3)" /> is the 
set of all <img src="https://latex.codecogs.com/svg.image?3\times3" title="https://latex.codecogs.com/svg.image?3\times3" /> real matrices R satisfying:
 - <img src="https://latex.codecogs.com/svg.image?R^TR=I" title="https://latex.codecogs.com/svg.image?R^TR=I" />
 - <img src="https://latex.codecogs.com/svg.image?det&space;R=1" title="https://latex.codecogs.com/svg.image?det R=1" />



<img src="https://latex.codecogs.com/svg.image?so(3)" title="https://latex.codecogs.com/svg.image?so(3)" />: the set of all 3x3 skew-symmetric real matrices is called <img src="https://latex.codecogs.com/svg.image?so(3)" title="https://latex.codecogs.com/svg.image?so(3)" />


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;[\mathbf&space;{x}&space;]{=}{\begin{bmatrix}\,\,0&\!-x_{3}&\,\,\,x_{2}\\\,\,\,x_{3}&0&\!-x_{1}\\\!-x_{2}&\,\,x_{1}&\,\,0\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle [\mathbf {x} ]{=}{\begin{bmatrix}\,\,0&\!-x_{3}&\,\,\,x_{2}\\\,\,\,x_{3}&0&\!-x_{1}\\\!-x_{2}&\,\,x_{1}&\,\,0\end{bmatrix}}}" />

because:

<img src="https://latex.codecogs.com/svg.image?[x]=-[x]^T" title="https://latex.codecogs.com/svg.image?[x]=-[x]^T" />



## SE(2)
The set of all 3×3 matrices with the structure: 


<img src="https://latex.codecogs.com/svg.latex?T%3D%5Cbegin%7Bbmatrix%7D%20cos%28%5Ctheta%29%20%26%20sin%28%5Ctheta%29%20%26%20r_x%5C%5C%20sin%28%5Ctheta%29%20%26%20-cos%28%5Ctheta%29%20%26%20r_y%20%5C%5C%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?T=\begin{bmatrix}
cos(\theta) & sin(\theta) & r_x\\ 
sin(\theta) & -cos(\theta) & r_y \\
0 & 0 & 1
\end{bmatrix}" />


## SE(3)
The special Euclidean groups <img src="https://latex.codecogs.com/svg.latex?SE(3)" alt="https://latex.codecogs.com/svg.latex?SE(3)" /> is the set of all 4x4 real matrices T of the form

<img src="https://latex.codecogs.com/svg.latex?T%3D%5Cbegin%7Bbmatrix%7D%20R%20%26%20p%20%5C%5C%200%20%26%201%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%20r_%7B11%7D%20%26%20r_%7B12%7D%20%26%20r_%7B13%7D%20%26%20p_1%5C%5C%20r_%7B21%7D%20%26%20r_%7B22%7D%20%26%20r_%7B23%7D%20%26%20p_2%5C%5C%20r_%7B31%7D%20%26%20r_%7B32%7D%20%26%20r_%7B33%7D%20%26%20p_3%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?T=\begin{bmatrix}
R & p \\ 
0 & 1
\end{bmatrix}=\begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_1\\ 
r_{21} & r_{22} & r_{23} & p_2\\ 
r_{31} & r_{32} & r_{33} & p_3\\ 
0 & 0 & 0 & 1
\end{bmatrix}" />


<img src="https://latex.codecogs.com/svg.latex?R%5Cin%20SO%283%29%2C%20p%5Cin%20%5Cmathbb%7BR%7D%5E3" alt="https://latex.codecogs.com/svg.latex?R\in SO(3), p\in \mathbb{R}^3" />

<img src="https://latex.codecogs.com/svg.latex?T%5E%7B-1%7D%3D%5Cbegin%7Bbmatrix%7D%20R%20%26%20p%20%5C%5C%200%20%26%201%20%5Cend%7Bbmatrix%7D%5E%7B-1%7D%3D%5Cbegin%7Bbmatrix%7D%20R%5ET%20%26%20-R%5ETp%20%5C%5C%200%20%26%201%20%5Cend%7Bbmatrix%7D%5Cin%20SE%283%29" alt="T^{-1}=\begin{bmatrix}
R & p \\ 
0 & 1
\end{bmatrix}^{-1}=\begin{bmatrix}
R^T & -R^Tp \\ 
0 & 1
\end{bmatrix}\in SE(3)" />


<img src="https://latex.codecogs.com/svg.image?SE(3)" title="https://latex.codecogs.com/svg.image?SE(3)" />: the special Euclidean groups <img src="https://latex.codecogs.com/svg.image?SE(3)" title="https://latex.codecogs.com/svg.image?SE(3)" />
is the set of all <img src="https://latex.codecogs.com/svg.image?4&space;\times&space;4" title="https://latex.codecogs.com/svg.image?4 \times 4" /> real matrices of <img src="https://latex.codecogs.com/svg.image?T&space;" title="https://latex.codecogs.com/svg.image?T " /> f the form:

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?T=\begin{bmatrix}&space;R&&space;p&space;\\&space;0&&space;1&space;\\\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?T=\begin{bmatrix} R& p \\ 0& 1 \\\end{bmatrix} " />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?R\in&space;SO(3),&space;p\in&space;R^3" title="https://latex.codecogs.com/svg.image?R\in SO(3), p\in R^3" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?T^{-1=}\begin{bmatrix}&space;R^T&&space;-pR^T&space;\\&space;0&&space;1&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T^{-1=}\begin{bmatrix} R^T& -pR^T \\ 0& 1 \\\end{bmatrix}" />


Refs: [1](https://www.seas.upenn.edu/~meam620/notes/RigidBodyMotion3.pdf), [2](https://www.youtube.com/watch?v=NHXAnvv4mM8&list=PLdMorpQLjeXmbFaVku4JdjmQByHHqTd1F&index=13)





<img src="https://latex.codecogs.com/svg.image?&space;\hat{\omega_s}&space;" title="https://latex.codecogs.com/svg.image? \hat{\omega_s} " /> is a unit vector, if we rotate a frame around it at the rate of <img src="https://latex.codecogs.com/svg.image?\dot{\theta}&space;" title="https://latex.codecogs.com/svg.image?\dot{\theta} " />, the angular velocity is <img src="https://latex.codecogs.com/svg.image?\omega_s" title="https://latex.codecogs.com/svg.image?\omega_s" /> expresses in frame <img src="https://latex.codecogs.com/svg.image?s" title="https://latex.codecogs.com/svg.image?s" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\dot{R_{sb}}&space;=[\omega_s]R_{sb}" title="https://latex.codecogs.com/svg.image?\dot{R_{sb}} =[\omega_s]R_{sb}" />
<br/>
<br/>
<img src="images/angulare_velocity_2.jpg" width="350" height="200" />
<br/>
<br/>

Full configuration <img src="https://latex.codecogs.com/svg.image?T" title="https://latex.codecogs.com/svg.image?T" />



4. <img src="https://latex.codecogs.com/svg.image?se(3)" title="https://latex.codecogs.com/svg.image?se(3)" />: the set of all <img src="https://latex.codecogs.com/svg.image?4&space;\times&space;4" title="https://latex.codecogs.com/svg.image?4 \times 4" /> real matrices with a <img src="https://latex.codecogs.com/svg.image?3&space;\times&space;3" title="https://latex.codecogs.com/svg.image?3 \times 3" /> <img src="https://latex.codecogs.com/svg.image?so(3)" title="https://latex.codecogs.com/svg.image?so(3)" /> matrix at top left and four zeros in the bottom row is classed <img src="https://latex.codecogs.com/svg.image?se(3)" title="https://latex.codecogs.com/svg.image?se(3)" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\dot{T_{sb}}T_{sb}^{-1}=\begin{bmatrix}&space;[\omega_s]&space;&&space;v_s&space;\\&space;0&&space;&space;0\\\end{bmatrix}&space;=[\nu_s]\in&space;se(3)" title="https://latex.codecogs.com/svg.image?\dot{T_{sb}}T_{sb}^{-1}=\begin{bmatrix} [\omega_s] & v_s \\ 0& 0\\\end{bmatrix} =[\nu_s]\in se(3)" />




# Angular Velocity







<img src="https://latex.codecogs.com/svg.image?\omega=\frac{\theta}{t}" title="https://latex.codecogs.com/svg.image?\omega=\frac{\theta}{t}" />

# Linear Velocity 


<img src="https://latex.codecogs.com/svg.image?v=\frac{s}{t}=\frac{r\theta}{t}&space;&space;&space;=r\omega" title="https://latex.codecogs.com/svg.image?v=\frac{s}{t}=\frac{r\theta}{t} =r\omega" />

<br/>
<br/>

<img src="images/linear_velocity.png" />

# Screw
screw is axis

<img src="https://latex.codecogs.com/svg.image?s=\begin{bmatrix}s_{\omega}&space;\\s_{v}\end{bmatrix}=\begin{bmatrix}\text{angulare&space;velocity&space;when&space;}\dot{\theta}=1&space;&space;\\\text{linear&space;velocity&space;of&space;the&space;origin&space;when&space;}\dot{\theta}=1&space;\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?s=\begin{bmatrix}s_{\omega} \\s_{v}\end{bmatrix}=\begin{bmatrix}\text{angulare velocity when }\dot{\theta}=1 \\\text{linear velocity of the origin when }\dot{\theta}=1 \end{bmatrix} " />

The linear velocity of origins is combination of two terms:
- <img src="https://latex.codecogs.com/svg.image?h\hat{s}" title="https://latex.codecogs.com/svg.image?h\hat{s}" /> which coming from linear move

- <img src="https://latex.codecogs.com/svg.image?-\hat{s}\times&space;q" title="https://latex.codecogs.com/svg.image?-\hat{s}\times q" />

<video width="640" height="480" controls>
  <source src="vidoes/rotation_translation.mp4" type="video/mp4">
</video>


<img src="images/screw_1.jpg" width="400" height="200"/>

<br/>
<br/>




# Twist
twist is full representation of linear and angular velocity:

<img src="https://latex.codecogs.com/svg.image?\nu=\begin{bmatrix}\omega&space;\\&space;v\end{bmatrix}_{6\times1}=s\dot\theta" title="https://latex.codecogs.com/svg.image?\nu=\begin{bmatrix}\omega \\ v\end{bmatrix}_{6\times1}=s\dot\theta" />


<br/>
<br/>

The <img src="https://latex.codecogs.com/svg.image?6\times&space;6" title="https://latex.codecogs.com/svg.image?6\times 6" /> adjoint representation of a transformation matrix 


<img src="https://latex.codecogs.com/svg.image?T=\begin{bmatrix}R&space;&&space;p&space;\\0&space;&&space;1&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T=\begin{bmatrix}R & p \\0 & 1 \\\end{bmatrix}" /> is <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}Ad_T\end{bmatrix}=\begin{bmatrix}R&space;&&space;0&space;\\&space;[p]R&space;&&space;R&space;\\\end{bmatrix}\in&space;\mathbb{R}^{6\times6}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}Ad_T\end{bmatrix}=\begin{bmatrix}R & 0 \\ [p]R & R \\\end{bmatrix}\in \mathbb{R}^{6\times6}" />

which enable us the subscribe cancaltion 

<img src="https://latex.codecogs.com/svg.image?\nu_a=[Ad_T_{ab}]\nu_{b}" title="https://latex.codecogs.com/svg.image?\nu_a=[Ad_T_{ab}]\nu_{b}" />
for chaining the frame of reference

<br/>
<br/>




For angular velocity we had:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\dot{R_{sb}}&space;=[\omega_s]R_{sb}" title="https://latex.codecogs.com/svg.image?\dot{R_{sb}} =[\omega_s]R_{sb}" />




<img src="https://latex.codecogs.com/svg.image?\dot{R_{sb}}R_{sb}^{-1}&space;=[\omega_s]\in&space;so(3)" title="https://latex.codecogs.com/svg.image?\dot{R_{sb}}R_{sb}^{-1} =[\omega_s]\in so(3)" />
<br/>
<br/>
similarly for twist we have:


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\dot{T_{sb}}T_{sb}^{-1}=\begin{bmatrix}&space;[\omega_s]&space;&&space;v_s&space;\\&space;0&&space;&space;0\\\end{bmatrix}&space;=[\nu_s]\in&space;se(3)" title="https://latex.codecogs.com/svg.image?\dot{T_{sb}}T_{sb}^{-1}=\begin{bmatrix} [\omega_s] & v_s \\ 0& 0\\\end{bmatrix} =[\nu_s]\in se(3)" />



## Representing Robot Pose


<img src="images/representing_robot_pose1.png" width="50%" height="50%" />

Refs: [1](https://web.archive.org/web/20161029231029/https://paulfurgale.info/news/2014/6/9/representing-robot-pose-the-good-the-bad-and-the-ugly)

