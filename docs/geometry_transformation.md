# Geometry Transformation

# 1. Euler Angles
## 1.1. Introduction
The Euler angles are three angles to describe the orientation of a rigid body with respect to a fixed coordinate system.



The rotations may be about the axes `XYZ` of the original coordinate system, which is assumed to remain motionless (extrinsic), or rotations about the axes of the rotating coordinate system `XYZ` (intrinsic), solidary with the moving body, which changes its orientation with respect to the extrinsic frame after each elemental rotation.


## 1.2. Roll, Pitch, and Yaw

Euler angles are typically denoted as:
- <img src="https://latex.codecogs.com/svg.image?&space;\gamma&space;\text{&space;or&space;}&space;\phi,&space;" title="https://latex.codecogs.com/svg.image? \gamma \text{ or } \phi, " /> represents a rotation around the x axis.
- <img src="https://latex.codecogs.com/svg.image?\beta,&space;\text{&space;or&space;}&space;\theta" title="https://latex.codecogs.com/svg.image?\beta, \text{ or } \theta" /> represents a rotation around the y axis,
- <img src="https://latex.codecogs.com/svg.image?\alpha&space;\text{&space;or&space;}&space;\psi" title="https://latex.codecogs.com/svg.image?\alpha \text{ or } \psi" /> represents a rotation around the z axis,








## 1.3. Proper Euler angles and Tait-Bryan angles

There exist twelve possible sequences of rotation axes, which can be divided into two categories: 
1. **Proper Euler angles**, where one axis of rotation is repeated (x-z-x, x-y-x, y-x-y, y-z-y, z-y-z, z-x-z), 
2. **Tait-Bryan angles**, which rotate around all axes (x-z-y, x-y-z, y-x-z, y-z-x, z-y-x, z-x-y).



Sometimes, both kinds of sequences are called "Euler angles". In that case, the sequences of the first group are called **proper** or **classic Euler** angles.


There are six possibilities of choosing the rotation axes for Tait–Bryan angles. The six possible sequences are:

- x-y′-z″ (intrinsic rotations) or z-y-x (extrinsic rotations)
- y-z′-x″ (intrinsic rotations) or x-z-y (extrinsic rotations)
- z-x′-y″ (intrinsic rotations) or y-x-z (extrinsic rotations)
- x-z′-y″ (intrinsic rotations) or y-z-x (extrinsic rotations)
- z-y′-x″ (intrinsic rotations) or x-y-z (extrinsic rotations): the intrinsic rotations are known as: yaw, pitch and roll
- y-x′-z″ (intrinsic rotations) or z-x-y (extrinsic rotations)

## 1.4. Rotation matrix

<img src="https://latex.codecogs.com/svg.latex?R_z%28%5Calpha%29%3D%5Cbegin%7Bpmatrix%7D%20cos%28%5Calpha%29%20%26%20-sin%28%5Calpha%29%20%260%20%5C%5C%20sin%28%5Calpha%29%20%26%20cos%28%5Calpha%29%20%260%20%5C%5C%200%20%26%200%20%26%201%5C%5C%20%5Cend%7Bpmatrix%7D" title="https://latex.codecogs.com/svg.image?R_z(\alpha)=\begin{pmatrix} cos(\alpha) & -sin(\alpha) &0 \\  sin(\alpha) & cos(\alpha) &0 \\ 0 & 0 & 1\\ \end{pmatrix}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?R_y(\beta)=\bigl(\begin{smallmatrix}cos(\beta)&space;&&space;0&space;&&space;sin(\beta)&space;\\&space;0&space;&&space;&space;1&space;&0&space;\\&space;&space;&space;-sin(\beta)&space;&space;&space;&space;&space;&space;&space;&space;&space;&space;&space;&&space;&space;&space;&space;0&space;&space;&space;&&space;&space;cos(\beta)\\&space;\end{smallmatrix}\bigr)" title="https://latex.codecogs.com/svg.image?R_y(\beta)=\bigl(\begin{smallmatrix}cos(\beta) & 0 & sin(\beta) \\ 0 & 1 &0 \\ -sin(\beta) & 0 & cos(\beta)\\ \end{smallmatrix}\bigr)" />

<br/>
<br/>





<img src="https://latex.codecogs.com/svg.image?R(\alpha,\beta,&space;\gamma)=\bigl(\begin{smallmatrix}cos(\alpha)cos(&space;\beta)&space;&space;&&&space;&space;cos(\alpha)sin(\beta)sin(\gamma)&space;-sin(\alpha)cos(\gamma)&space;&space;&&&space;cos(\alpha)sin(\beta)cos(\gamma)&space;&plus;&space;sin(\alpha)sin(\gamma)&space;\\sin(\alpha)cos(&space;\beta)&space;&&&space;sin(\alpha)sin(\beta)sin(\gamma)&space;&plus;&space;cos(\alpha)cos(\gamma)&space;&&&space;sin(\alpha)sin(\beta)&space;cos(\gamma)&space;-&space;cos(&space;\alpha)sin(\gamma)&space;\\-sin(\beta)&space;&&&space;\cos(\beta)sin(\gamma)&space;&&&space;\cos(\beta)cos(\gamma)\end{smallmatrix}\bigr)" title="https://latex.codecogs.com/svg.image?R(\alpha,\beta, \gamma)=\bigl(\begin{smallmatrix}cos(\alpha)cos( \beta) && cos(\alpha)sin(\beta)sin(\gamma) -sin(\alpha)cos(\gamma) && cos(\alpha)sin(\beta)cos(\gamma) + sin(\alpha)sin(\gamma) \\sin(\alpha)cos( \beta) && sin(\alpha)sin(\beta)sin(\gamma) + cos(\alpha)cos(\gamma) && sin(\alpha)sin(\beta) cos(\gamma) - cos( \alpha)sin(\gamma) \\-sin(\beta) && \cos(\beta)sin(\gamma) && \cos(\beta)cos(\gamma)\end{smallmatrix}\bigr)" />




<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{aligned}R=R_{z}(\alpha&space;)\,R_{y}(\beta&space;)\,R_{x}(\gamma&space;)&={\overset&space;{\text{yaw}}{\begin{bmatrix}\cos&space;\alpha&space;&-\sin&space;\alpha&space;&0\\\sin&space;\alpha&space;&\cos&space;\alpha&space;&0\\0&0&1\\\end{bmatrix}}}{\overset&space;{\text{pitch}}{\begin{bmatrix}\cos&space;\beta&space;&0&\sin&space;\beta&space;\\0&1&0\\-\sin&space;\beta&space;&0&\cos&space;\beta&space;\\\end{bmatrix}}}{\overset&space;{\text{roll}}{\begin{bmatrix}1&0&0\\0&\cos&space;\gamma&space;&-\sin&space;\gamma&space;\\0&\sin&space;\gamma&space;&\cos&space;\gamma&space;\\\end{bmatrix}}}\\&={\begin{bmatrix}\cos&space;\alpha&space;\cos&space;\beta&space;&\cos&space;\alpha&space;\sin&space;\beta&space;\sin&space;\gamma&space;-\sin&space;\alpha&space;\cos&space;\gamma&space;&\cos&space;\alpha&space;\sin&space;\beta&space;\cos&space;\gamma&space;&plus;\sin&space;\alpha&space;\sin&space;\gamma&space;\\\sin&space;\alpha&space;\cos&space;\beta&space;&\sin&space;\alpha&space;\sin&space;\beta&space;\sin&space;\gamma&space;&plus;\cos&space;\alpha&space;\cos&space;\gamma&space;&\sin&space;\alpha&space;\sin&space;\beta&space;\cos&space;\gamma&space;-\cos&space;\alpha&space;\sin&space;\gamma&space;\\-\sin&space;\beta&space;&\cos&space;\beta&space;\sin&space;\gamma&space;&\cos&space;\beta&space;\cos&space;\gamma&space;\\\end{bmatrix}}\end{aligned}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{aligned}R=R_{z}(\alpha )\,R_{y}(\beta )\,R_{x}(\gamma )&={\overset {\text{yaw}}{\begin{bmatrix}\cos \alpha &-\sin \alpha &0\\\sin \alpha &\cos \alpha &0\\0&0&1\\\end{bmatrix}}}{\overset {\text{pitch}}{\begin{bmatrix}\cos \beta &0&\sin \beta \\0&1&0\\-\sin \beta &0&\cos \beta \\\end{bmatrix}}}{\overset {\text{roll}}{\begin{bmatrix}1&0&0\\0&\cos \gamma &-\sin \gamma \\0&\sin \gamma &\cos \gamma \\\end{bmatrix}}}\\&={\begin{bmatrix}\cos \alpha \cos \beta &\cos \alpha \sin \beta \sin \gamma -\sin \alpha \cos \gamma &\cos \alpha \sin \beta \cos \gamma +\sin \alpha \sin \gamma \\\sin \alpha \cos \beta &\sin \alpha \sin \beta \sin \gamma +\cos \alpha \cos \gamma &\sin \alpha \sin \beta \cos \gamma -\cos \alpha \sin \gamma \\-\sin \beta &\cos \beta \sin \gamma &\cos \beta \cos \gamma \\\end{bmatrix}}\end{aligned}}}" />




<br/>
<br/>

It is important to note that <img src="https://latex.codecogs.com/svg.image?&space;R(\alpha,\beta,\gamma)" title="https://latex.codecogs.com/svg.image? R(\alpha,\beta,\gamma)" /> performs the roll first, then the pitch, and finally the yaw. 

<br/>
<br/>

## 1.5. Determining Yaw, Pitch, And Roll From a Rotation Matrix
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{pmatrix}r_{11}&space;&&space;r_{12}&space;&r_{13}&space;&space;\\r_{21}&space;&&space;r_{22}&space;&&space;r_{23}&space;\\r_{31}&space;&&space;r_{32}&space;&&space;r_{33}&space;\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\begin{pmatrix}r_{11} & r_{12} &r_{13} \\r_{21} & r_{22} & r_{23} \\r_{31} & r_{32} & r_{33} \\\end{pmatrix}" />

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?tan(\alpha)=&space;\frac{r_{21}}{r_{11}}" title="https://latex.codecogs.com/svg.image?tan(\alpha)= \frac{r_{21}}{r_{11}}" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?tan(\gamma)=&space;\frac{r_{32}}{r_{33}}" title="https://latex.codecogs.com/svg.image?tan(\gamma)= \frac{r_{32}}{r_{33}}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?-sin(\beta)=r_{31}" title="https://latex.codecogs.com/svg.image?-sin(\beta)=r_{31}" />

and

<img src="https://latex.codecogs.com/svg.image?cos(\beta)=\sqrt{&space;&space;r_{32}^2&plus;&space;r_{33}^2}" title="https://latex.codecogs.com/svg.image?cos(\beta)=\sqrt{ r_{32}^2+ r_{33}^2}" />
<br/>
<br/>
There is a choice of four quadrants for the inverse tangent functions. Each quadrant should be chosen by using
the signs of the numerator and denominator of the argument. The <b>numerator</b> sign selects whether the direction will be above or below the <b> x-axis </b>, and the <b>denominator </b> selects whether the direction will be to the left or right of the <b>y-axis </b>. the function <b>atan2</b>
can calculate this for us:

- <img src="https://latex.codecogs.com/svg.image?\alpha=atan2(r_{21},r_{11})" title="https://latex.codecogs.com/svg.image?\alpha=atan2(r_{21},r_{11})" />

- <img src="https://latex.codecogs.com/svg.image?\beta=atan2(-r_{31},&space;\sqrt{&space;r_{32}^2&plus;&space;r_{33}^2})" title="https://latex.codecogs.com/svg.image?\beta=atan2(-r_{31}, \sqrt{ r_{32}^2+ r_{33}^2})" />

- <img src="https://latex.codecogs.com/svg.image?\gamma=atan2(-r_{32},r_{33})" title="https://latex.codecogs.com/svg.image?\gamma=atan2(-r_{32},r_{33})" />

Note that this method assumes <img src="https://latex.codecogs.com/svg.image?r_{11}\neq&space;0" title="https://latex.codecogs.com/svg.image?r_{11}\neq 0" /> and <img src="https://latex.codecogs.com/svg.image?r_{33}\neq&space;0" title="https://latex.codecogs.com/svg.image?r_{33}\neq 0" />.


## 1.6. Signs and ranges

- for <img src="https://latex.codecogs.com/svg.image?\alpha&space;\text{&space;or&space;}&space;\psi" title="https://latex.codecogs.com/svg.image?\alpha \text{ or } \psi" /> and <img src="https://latex.codecogs.com/svg.image?&space;\gamma&space;\text{&space;or&space;}&space;\phi,&space;" title="https://latex.codecogs.com/svg.image? \gamma \text{ or } \phi, " />, the range is defined modulo <img src="https://latex.codecogs.com/svg.image?2\pi" title="https://latex.codecogs.com/svg.image?2\pi" />radians. For instance, a valid range could be <img src="https://latex.codecogs.com/svg.image?[-\pi,&space;\pi]" title="https://latex.codecogs.com/svg.image?[-\pi, \pi]" />.
- for <img src="https://latex.codecogs.com/svg.image?\beta,&space;\text{&space;or&space;}&space;\theta" title="https://latex.codecogs.com/svg.image?\beta, \text{ or } \theta" />, the range covers <img src="https://latex.codecogs.com/svg.image?&space;\pi&space;" title="https://latex.codecogs.com/svg.image? \pi " /> radians (but can't be said to be modulo <img src="https://latex.codecogs.com/svg.image?&space;\pi&space;" title="https://latex.codecogs.com/svg.image? \pi " />). For example, it could be <img src="https://latex.codecogs.com/svg.image?[0,&space;\pi]" title="https://latex.codecogs.com/svg.image?[0, \pi]" /> or <img src="https://latex.codecogs.com/svg.image?[-\pi/2,&space;\pi/2]" title="https://latex.codecogs.com/svg.image?[-\pi/2, \pi/2]" />.




## 1.7. Tait–Bryan Angles
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;Z_{1}Y_{2}X_{3}}" title="https://latex.codecogs.com/svg.image?{\displaystyle Z_{1}Y_{2}X_{3}}" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?{\begin{aligned}\alpha&space;&=\arctan&space;\left({\frac&space;{r_{21}}{r_{11}}}\right)\\\beta&space;&=\arctan&space;\left({\frac&space;{-r_{31}}{\sqrt&space;{1-r_{31}^{2}}}}\right)\\\gamma&space;&=\arctan&space;\left({\frac&space;{r_{32}}{r_{33}}}\right)\end{aligned}}" title="https://latex.codecogs.com/svg.image?{\begin{aligned}\alpha &=\arctan \left({\frac {r_{21}}{r_{11}}}\right)\\\beta &=\arctan \left({\frac {-r_{31}}{\sqrt {1-r_{31}^{2}}}}\right)\\\gamma &=\arctan \left({\frac {r_{32}}{r_{33}}}\right)\end{aligned}}" />
<br/>
<br/>

## 1.8. Equivalent Proper Euler Angles

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;Z_{1}Y_{2}Z_{3}}" title="https://latex.codecogs.com/svg.image?{\displaystyle Z_{1}Y_{2}Z_{3}}" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?{\begin{aligned}\alpha&space;&=\arctan&space;\left({\frac&space;{r_{23}}{r_{13}}}\right)\\\beta&space;&=\arctan&space;\left({\frac&space;{\sqrt&space;{1-r_{33}^{2}}}{r_{33}}}\right)\\\gamma&space;&=\arctan&space;\left({\frac&space;{r_{32}}{-r_{31}}}\right)\end{aligned}}" title="https://latex.codecogs.com/svg.image?{\begin{aligned}\alpha &=\arctan \left({\frac {r_{23}}{r_{13}}}\right)\\\beta &=\arctan \left({\frac {\sqrt {1-r_{33}^{2}}}{r_{33}}}\right)\\\gamma &=\arctan \left({\frac {r_{32}}{-r_{31}}}\right)\end{aligned}}" />





# 1.9. Gimbal Lock 
The angles <img src="https://latex.codecogs.com/svg.image?\alpha,&space;\beta,&space;\text{&space;and&space;}&space;\gamma" title="https://latex.codecogs.com/svg.image?\alpha, \beta, \text{ and } \gamma" /> are uniquely determined except for the singular case. If <img src="https://latex.codecogs.com/svg.image?cos(\beta)=0&space;\text{&space;or&space;}&space;\beta=\pm&space;\pi/2" title="https://latex.codecogs.com/svg.image?cos(\beta)=0 \text{ or } \beta=\pm \pi/2" />

1. <img src="https://latex.codecogs.com/svg.image?\beta=\pi/2" title="https://latex.codecogs.com/svg.image?\beta=\pi/2" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{pmatrix}0&space;&&space;sin(\gamma-\alpha)&space;&&space;cos(\gamma-\alpha)\\0&space;&&space;cos(\gamma-\alpha)&space;&&space;-sin(\gamma-\alpha)\\-1&space;&&space;0&space;&&space;0\end{pmatrix}&space;=&space;\begin{pmatrix}0&space;&&space;r_{12}&space;&&space;r_{13}\\0&space;&&space;r_{22}&space;&&space;r_{23}\\-1&space;&&space;0&space;&&space;0\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\begin{pmatrix}0 & sin(\gamma-\alpha) & cos(\gamma-\alpha)\\0 & cos(\gamma-\alpha) & -sin(\gamma-\alpha)\\-1 & 0 & 0\end{pmatrix} = \begin{pmatrix}0 & r_{12} & r_{13}\\0 & r_{22} & r_{23}\\-1 & 0 & 0\end{pmatrix}" />

This will result in:

- <img src="https://latex.codecogs.com/svg.image?\gamma-\alpha&space;=&space;atan2(r_{12},r_{22})" title="https://latex.codecogs.com/svg.image?\gamma-\alpha = atan2(r_{12},r_{22})" />

- <img src="https://latex.codecogs.com/svg.image?\gamma-\alpha&space;=&space;atan2(-r_{23},r_{13})&space;" title="https://latex.codecogs.com/svg.image?\gamma-\alpha = atan2(-r_{23},r_{13}) " />







<video width="640" height="480" controls>
  <source src="vidoes/gimbal_locl_beta_pi_2.mp4" type="video/mp4">
</video>

















2. <img src="https://latex.codecogs.com/svg.image?\beta=-\pi/2" title="https://latex.codecogs.com/svg.image?\beta=-\pi/2" />

<img src="https://latex.codecogs.com/svg.image?\begin{pmatrix}0&space;&&space;-sin(\gamma&plus;\alpha)&space;&&space;-cos(\gamma&plus;\alpha)\\0&space;&&space;cos(\gamma&plus;\alpha)&space;&&space;-sin(\gamma&plus;\alpha)\\1&space;&&space;0&space;&&space;0\end{pmatrix}&space;=&space;\begin{pmatrix}0&space;&&space;r_{12}&space;&&space;r_{13}\\0&space;&&space;r_{22}&space;&&space;r_{23}\\1&space;&&space;0&space;&&space;0\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\begin{pmatrix}0 & -sin(\gamma+\alpha) & -cos(\gamma+\alpha)\\0 & cos(\gamma+\alpha) & -sin(\gamma+\alpha)\\1 & 0 & 0\end{pmatrix} = \begin{pmatrix}0 & r_{12} & r_{13}\\0 & r_{22} & r_{23}\\1 & 0 & 0\end{pmatrix}" />


This will result in:

- <img src="https://latex.codecogs.com/svg.image?\alpha&space;&plus;\gamma&space;=&space;atan2(-r_{12},r_{22})" title="https://latex.codecogs.com/svg.image?\alpha +\gamma = atan2(-r_{12},r_{22})" />

- <img src="https://latex.codecogs.com/svg.image?\gamma&plus;\alpha&space;=&space;atan2(-r_{23},-r_{13})" title="https://latex.codecogs.com/svg.image?\gamma+\alpha = atan2(-r_{23},-r_{13})" />

This means that there are infinitely many sets of (roll,yaw) angles for a given rotation matrix at with <img src="https://latex.codecogs.com/svg.image?\beta=\pm&space;\pi/2" alt="https://latex.codecogs.com/svg.image?\beta=\pm \pi/2" />


Visit the [link](https://compsci290-s2016.github.io/CoursePage/Materials/EulerAnglesViz/) for interactive Gimbal visualization.




Absolutely, let's illustrate the gimbal lock issue using a numerical example and then explain how the problem manifests in the Euler angle representation but not with quaternions.

### Numerical Example:

Consider a 3D object that we wish to rotate using the roll-pitch-yaw sequence (often used in aerospace). For the sake of simplicity, let's work with degrees:

1. **Initial orientation**: No rotation applied. Euler angles are (roll, pitch, yaw) = (0°, 0°, 0°).
   
2. **Rotation**: We apply a pitch of +90°. Now, our Euler angles are (0°, 90°, 0°).

At this point, the object's 'nose' is pointing straight up. Here's the problem:

If we now try to apply a roll of, say, +45°, the actual effect in 3D space will be identical to applying a yaw of +45°. We cannot distinguish between roll and yaw anymore; they have become degenerate. This is gimbal lock. 

### Numerical Values:

**Euler Angles**:
After the `+90°` pitch, our Euler angles become:
Roll: `0°` (or `+45°` if we attempt a roll after pitching)
Pitch: `90°`
Yaw: `0°` (or `+45°` if we attempt a yaw after pitching)

This is problematic because after the pitch of +90°, the roll and yaw rotations are indistinguishable in effect.








**Quaternion Representation**:
The rotation for a +90° pitch around the Y-axis can be represented as:
 <img src="https://latex.codecogs.com/svg.latex?q%20%3D%20%5Ccos%5Cleft%28%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Cright%29%20&plus;%20%5Csin%5Cleft%28%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Cright%29%5Cmathbf%7Bj%7D" alt="q = \cos\left(\frac{\theta}{2}\right) + \sin\left(\frac{\theta}{2}\right)\mathbf{j}" />  
= <img src="https://latex.codecogs.com/svg.latex?q%20%3D%20%5Ccos%2845%29%20&plus;%20%5Csin%2845%29%5Cmathbf%7Bj%7D" alt="q = \cos(45) + \sin(45)\mathbf{j}" />  

= <img src="https://latex.codecogs.com/svg.latex?q%20%3D%200.707%20&plus;%200.707%5Cmathbf%7Bj%7D" alt="q = 0.707 + 0.707\mathbf{j}" />  
 


Now, if we wanted to apply a roll of +45° after this pitch using quaternions, we would multiply the above quaternion by the quaternion representation of a +45° roll around the X-axis, resulting in a distinct and unique quaternion value that smoothly combines both rotations without ambiguity.

### Why Euler Angles Have This Problem:

The core of the gimbal lock problem with Euler angles lies in the sequential nature of the rotations. When the pitch angle is ±90°, the axes for roll and yaw become aligned. Hence, rotating around one of these axes is indistinguishable from rotating around the other. This overlap or "lock" is what causes the loss of a degree of freedom.

### Why Quaternions Don't Have This Problem:

Quaternions represent rotations as a single, unified operation rather than a sequence. This means there's no inherent order or sequence to worry about. A quaternion rotation of +90° pitch followed by a +45° roll will result in a unique orientation distinct from any other combination of rotations. 

Furthermore, quaternions interpolate smoothly between orientations using "slerp" (spherical linear interpolation), ensuring a consistent and continuous rotation without the jumps or singularities associated with Euler angles.

In summary, the non-sequential nature of quaternions, combined with their ability to uniquely represent every possible orientation in 3D space, makes them immune to the gimbal lock problem that plagues Euler angles.



<img src="images/quaternions.online.png" width="100%"  height="100%" alt="quaternions.online.png"  />


Click here for [interactive](https://quaternions.online/) demo



## 1.10. Uniqueness of 3D Rotation Matrix

Refs: [1](https://math.stackexchange.com/questions/105264/3d-rotation-matrix-uniqueness/105380#105380)

# 2. Global References and Local Tangent Plane Coordinates


There are several axes conventions in practice for choosing the mobile and fixed axes and these conventions determine the signs of the angles.


Tait–Bryan angles are often used to describe a vehicle's attitude with respect to a chosen reference frame. The positive x-axis in vehicles points always in the direction of movement. For positive y- and z-axis, we have to face two different conventions:
## 2.1. East, North, Up (ENU)
East, North, Up (ENU), used in geography (z is up and x is in the direction of move, y is pointing left)

## 2.2 North, East, Down (NED) 
- North, East, Down (NED), used specially in aerospace (z is down and x is in the direction of move, y is pointing right)



In case of land vehicles like cars, tanks  ENU-system (East-North-Up) as external reference (World frame), the vehicle's (body's) positive y- or pitch axis always points to its left, and the positive z- or yaw axis always points up.

<img src="images/RPY_angles_of_cars.png" width="250" height="150" />


In case of air and sea vehicles like submarines, ships, airplanes etc., which use the NED-system (North-East-Down) as external reference (World frame), the vehicle's (body's) positive y- or pitch axis always points to its right, and its positive z- or yaw axis always points down. 


<img src="images/RPY_angles_of_airplanes.png" width="250" height="150" />




<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Barray%7D%7Brcl%7D%20%5Cmathbf%7Bg%7D%20%26%3D%26%20%5Cleft%5C%7B%20%5Cbegin%7Barray%7D%7Bll%7D%20%5Cbegin%7Bbmatrix%7D0%20%26%200%20%26%20-1%5Cend%7Bbmatrix%7D%5ET%20%26%20%5Cmathrm%7Bif%7D%5C%3B%20%5Cmathrm%7BNED%7D%20%5C%5C%20%5Cbegin%7Bbmatrix%7D0%20%26%200%20%26%201%5Cend%7Bbmatrix%7D%5ET%20%26%20%5Cmathrm%7Bif%7D%5C%3B%20%5Cmathrm%7BENU%7D%20%5Cend%7Barray%7D%20%5Cright.%5C%5C%20%26%26%20%5C%5C%20%5Cmathbf%7Br%7D%20%26%3D%26%20%5Cleft%5C%7B%20%5Cbegin%7Barray%7D%7Bll%7D%20%5Cfrac%7B1%7D%7B%5Csqrt%7B%5Ccos%5E2%5Ctheta&plus;%5Csin%5E2%5Ctheta%7D%7D%5Cbegin%7Bbmatrix%7D%5Ccos%5Ctheta%20%26%200%20%26%20%5Csin%5Ctheta%5Cend%7Bbmatrix%7D%5ET%20%26%20%5Cmathrm%7Bif%7D%5C%3B%20%5Cmathrm%7BNED%7D%20%5C%5C%20%5Cfrac%7B1%7D%7B%5Csqrt%7B%5Ccos%5E2%5Ctheta&plus;%5Csin%5E2%5Ctheta%7D%7D%5Cbegin%7Bbmatrix%7D0%20%26%20%5Ccos%5Ctheta%20%26%20-%5Csin%5Ctheta%5Cend%7Bbmatrix%7D%5ET%20%26%20%5Cmathrm%7Bif%7D%5C%3B%20%5Cmathrm%7BENU%7D%20%5Cend%7Barray%7D%20%5Cright.%20%5Cend%7Barray%7D" alt="https://latex.codecogs.com/svg.latex?\begin{array}{rcl}
\mathbf{g} &=&
\left\{
\begin{array}{ll}
    \begin{bmatrix}0 & 0 & -1\end{bmatrix}^T & \mathrm{if}\; \mathrm{NED} \\
    \begin{bmatrix}0 & 0 & 1\end{bmatrix}^T & \mathrm{if}\; \mathrm{ENU}
\end{array}
\right.\\ && \\
\mathbf{r} &=&
\left\{
\begin{array}{ll}
    \frac{1}{\sqrt{\cos^2\theta+\sin^2\theta}}\begin{bmatrix}\cos\theta & 0 & \sin\theta\end{bmatrix}^T & \mathrm{if}\; \mathrm{NED} \\
    \frac{1}{\sqrt{\cos^2\theta+\sin^2\theta}}\begin{bmatrix}0 & \cos\theta & -\sin\theta\end{bmatrix}^T & \mathrm{if}\; \mathrm{ENU}
\end{array}
\right.
\end{array}" />



# 3. Axis-angle Representation

Axis-angle representation of a rotation in a three-dimensional Euclidean space by two quantities: 
1. A unit vector <img src="https://latex.codecogs.com/svg.image?\bold{e}" title="https://latex.codecogs.com/svg.image?\bold{e}" /> indicating the direction of an axis of rotation, 
2. An angle <img src="https://latex.codecogs.com/svg.image?\theta" title="https://latex.codecogs.com/svg.image?\theta" />

<img src="images/Angle_axis_vector.svg" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;(\mathrm&space;{axis}&space;,\mathrm&space;{angle}&space;)=\left({\begin{bmatrix}e_{x}\\e_{y}\\e_{z}\end{bmatrix}},\theta&space;\right)=\left({\begin{bmatrix}0\\0\\1\end{bmatrix}},{\frac&space;{\pi&space;}{2}}\right).}" title="https://latex.codecogs.com/svg.image?{\displaystyle (\mathrm {axis} ,\mathrm {angle} )=\left({\begin{bmatrix}e_{x}\\e_{y}\\e_{z}\end{bmatrix}},\theta \right)=\left({\begin{bmatrix}0\\0\\1\end{bmatrix}},{\frac {\pi }{2}}\right).}" />

The above example can be represented as:

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}0\\0\\{\frac&space;{\pi&space;}{2}}\end{bmatrix}}.}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}0\\0\\{\frac {\pi }{2}}\end{bmatrix}}.}" />

## Rodrigues' Rotation Formula

If <img src="https://latex.codecogs.com/svg.image?\mathbf{v}" title="https://latex.codecogs.com/svg.image?\mathbf{v}" /> is a vector in <img src="https://latex.codecogs.com/svg.image?\mathbb{R}^3" title="https://latex.codecogs.com/svg.image?\mathbb{R}^3" /> and <img src="https://latex.codecogs.com/svg.image?\mathbf{k}" title="https://latex.codecogs.com/svg.image?\mathbf{k}" /> is a unit vector describing an axis of rotation by an angle <img src="https://latex.codecogs.com/svg.image?\theta" title="https://latex.codecogs.com/svg.image?\theta" />






<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;\mathbf&space;{v}&space;_{\mathrm&space;{rot}&space;}=\mathbf&space;{v}&space;\cos&space;\theta&space;&plus;(\mathbf&space;{k}&space;\times&space;\mathbf&space;{v}&space;)\sin&space;\theta&space;&plus;\mathbf&space;{k}&space;~(\mathbf&space;{k}&space;\cdot&space;\mathbf&space;{v}&space;)(1-\cos&space;\theta&space;)\,.}" title="https://latex.codecogs.com/svg.image?{\displaystyle \mathbf {v} _{\mathrm {rot} }=\mathbf {v} \cos \theta +(\mathbf {k} \times \mathbf {v} )\sin \theta +\mathbf {k} ~(\mathbf {k} \cdot \mathbf {v} )(1-\cos \theta )\,.}" />



<br/>
<br/>


to get the rotation matrix:


<img src="https://latex.codecogs.com/svg.image?\displaystyle&space;R=I&plus;(\sin&space;\theta&space;)\mathbf&space;{K}&space;&plus;(1-\cos&space;\theta&space;)\mathbf&space;{K}&space;^{2}\" title="https://latex.codecogs.com/svg.image?\displaystyle R=I+(\sin \theta )\mathbf {K} +(1-\cos \theta )\mathbf {K} ^{2}\" />


where <img src="https://latex.codecogs.com/svg.image?\mathbf{K}" title="https://latex.codecogs.com/svg.image?\mathbf{K}" /> is written in the matrix form.


<br/>
<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;[\mathbf&space;{K}&space;]_{\times&space;}{\stackrel&space;{\rm&space;{def}}{=}}{\begin{bmatrix}\,\,0&\!-k_{3}&\,\,\,k_{2}\\\,\,\,k_{3}&0&\!-k_{1}\\\!-k_{2}&\,\,k_{1}&\,\,0\end{bmatrix}}.}" title="https://latex.codecogs.com/svg.image?{\displaystyle [\mathbf {K} ]_{\times }{\stackrel {\rm {def}}{=}}{\begin{bmatrix}\,\,0&\!-k_{3}&\,\,\,k_{2}\\\,\,\,k_{3}&0&\!-k_{1}\\\!-k_{2}&\,\,k_{1}&\,\,0\end{bmatrix}}.}" />


## Exponential Coordinates For Rotation
Any orientation can be achieved from initial orientation by rotating about some unit axis <img src="https://latex.codecogs.com/svg.image?&space;\hat{\omega_s}&space;" title="https://latex.codecogs.com/svg.image? \hat{\omega_s} " /> (angular velocity)  by a particular angle <img src="https://latex.codecogs.com/svg.image?\theta" title="https://latex.codecogs.com/svg.image?\theta" />. If we multiply these two we will get <img src="https://latex.codecogs.com/svg.image?\hat{\omega}&space;\theta" title="https://latex.codecogs.com/svg.image?\hat{\omega} \theta" /> which is a three parameter representation of parameter. We call these three parameters **Exponential Coordinates** representing the orientation of one frame relative to another.

<br/>
<br/>
<img src="images/exponential_coordinates_for_rotation.png" with="300" height="260" />

<br/>
<br/>
The answer to this vector differential equation is matrix exponential which can be expressed with series expansion.
 
 
<br/>
<br/> 
 
 

 
 
<img src="https://latex.codecogs.com/svg.image?x(t)=e^{at}x(0)" title="https://latex.codecogs.com/svg.image?x(t)=e^{at}x(0)" />
 
<br/>
<br/>
 
 
<img src="https://latex.codecogs.com/svg.image?\dot{x(t)}&space;=&space;ax(t)" title="https://latex.codecogs.com/svg.image?\dot{x(t)} = ax(t)" />

<br/>
<br/>

## Taylor Series 



<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20f%28a%29&plus;%7B%5Cfrac%20%7Bf%27%28a%29%7D%7B1%21%7D%7D%28x-a%29&plus;%7B%5Cfrac%20%7Bf%27%27%28a%29%7D%7B2%21%7D%7D%28x-a%29%5E%7B2%7D&plus;%7B%5Cfrac%20%7Bf%27%27%27%28a%29%7D%7B3%21%7D%7D%28x-a%29%5E%7B3%7D&plus;%5Ccdots%20%2C%7D" alt="{\displaystyle f(a)+{\frac {f'(a)}{1!}}(x-a)+{\frac {f''(a)}{2!}}(x-a)^{2}+{\frac {f'''(a)}{3!}}(x-a)^{3}+\cdots ,}" />

if we write it for <img src="https://latex.codecogs.com/svg.image?x(t)=e^{at}x(0)" title="https://latex.codecogs.com/svg.image?x(t)=e^{at}x(0)" /> around point zero:

<br/>
<br/>



<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20f%280%29&plus;%7B%5Cfrac%20%7Bf%27%280%29%7D%7B1%21%7D%7D%28x-0%29&plus;%7B%5Cfrac%20%7Bf%27%27%280%29%7D%7B2%21%7D%7D%28x-0%29%5E%7B2%7D&plus;%7B%5Cfrac%20%7Bf%27%27%27%280%29%7D%7B3%21%7D%7D%28x-0%29%5E%7B3%7D&plus;%5Ccdots%20%2C%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle f(0)+{\frac {f'(0)}{1!}}(x-0)+{\frac {f''(0)}{2!}}(x-0)^{2}+{\frac {f'''(0)}{3!}}(x-0)^{3}+\cdots ,}" />

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%201&plus;%7B%5Cfrac%20a%7B1%21%7D%7Dx&plus;%7B%5Cfrac%20%7Ba%5E2%7D%7B2%21%7D%7Dx%5E%7B2%7D&plus;%7B%5Cfrac%20%7Ba%5E3%7D%7B3%21%7D%7Dx%5E%7B3%7D&plus;%5Ccdots%20%2C%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle 1+{\frac a{1!}}x+{\frac {a^2}{2!}}x^{2}+{\frac {a^3}{3!}}x^{3}+\cdots ,}" />





<br/>
<br/>
if <img src="https://latex.codecogs.com/svg.image?x\in&space;\mathbb{R}" title="https://latex.codecogs.com/svg.image?x\in \mathbb{R}" />
 
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?e^{at}=1&plus;at&plus;\frac{(at)^2}{2!}&plus;\frac{(at)^3}{3!}&plus;..." title="https://latex.codecogs.com/svg.image?e^{at}=1+at+\frac{(at)^2}{2!}+\frac{(at)^3}{3!}+..." />

<br/>
<br/>
and if <img src="https://latex.codecogs.com/svg.image?x\in&space;\mathbb{R}^n" title="https://latex.codecogs.com/svg.image?x\in \mathbb{R}^n" />
 

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?e^{At}=I&plus;At&plus;\frac{(At)^2}{2!}&plus;\frac{(At)^3}{3!}&plus;..." title="https://latex.codecogs.com/svg.image?e^{At}=I+At+\frac{(At)^2}{2!}+\frac{(At)^3}{3!}+..." />


<br/>
<br/>


when the matrix skew-symmetric the expansion has closed form solution:


<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\dot{p(t)}&space;=&space;[\hat{\omega}]p(t)" title="https://latex.codecogs.com/svg.image?\dot{p(t)} = [\hat{\omega}]p(t)" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?Rot(\hat{\omega},\theta&space;)=e^{[\hat{\omega}]\theta}=I&plus;sin\theta[\hat{\omega}]&plus;(1-cos\theta)[\hat{\omega}]^2\in&space;SO(3)" title="https://latex.codecogs.com/svg.image?Rot(\hat{\omega},\theta )=e^{[\hat{\omega}]\theta}=I+sin\theta[\hat{\omega}]+(1-cos\theta)[\hat{\omega}]^2\in SO(3)" />

<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?p(t)=e^{[\hat{\omega}]\theta}p(0)" title="https://latex.codecogs.com/svg.image?p(t)=e^{[\hat{\omega}]\theta}p(0)" />

<br/>
<br/>


## Exponential Coordinates For Rigid-Body Motions




<img src="images/exponential_coordinates_for_rigid-body_motions2.png" with="300" height="260" />

<br/>
<br/>
The final transformation of the frame:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?Tsb'=e^{[S_s]\theta}&space;Tsb" title="https://latex.codecogs.com/svg.image?Tsb'=e^{[S_s]\theta} Tsb" />


<br/>
<br/>

where if <img src="https://latex.codecogs.com/svg.image?S_w=0,&space;\lVert&space;S_v&space;\rVert=1" title="https://latex.codecogs.com/svg.image?S_w=0, \lVert S_v \rVert=1" /> then:


<img src="https://latex.codecogs.com/svg.image?e^{[S]\theta}&space;=\begin{bmatrix}I&space;&&space;S_v\theta&space;\\0&space;&&space;&space;&space;1\end{bmatrix}" title="https://latex.codecogs.com/svg.image?e^{[S]\theta} =\begin{bmatrix}I & S_v\theta \\0 & 1\end{bmatrix}" />

<br/>
<br/>

and if <img src="https://latex.codecogs.com/svg.image?\rVert&space;S_\omega&space;\rVert=1" title="https://latex.codecogs.com/svg.image?\rVert S_\omega \rVert=1" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?e^{[S]\theta}&space;=\begin{bmatrix}e^{[S_\omega]\theta}&space;&&space;(I\theta&plus;&space;(1-cos\theta)[S_\omega]&space;&plus;(\theta-sin\theta)[S_\omega]^2)S_v&space;\\0&space;&&space;&space;&space;1\end{bmatrix}" title="https://latex.codecogs.com/svg.image?e^{[S]\theta} =\begin{bmatrix}e^{[S_\omega]\theta} & (I\theta+ (1-cos\theta)[S_\omega] +(\theta-sin\theta)[S_\omega]^2)S_v \\0 & 1\end{bmatrix}" />


# 4. Quaternions

quaternion number system extends the complex numbers which introduced by William Rowan Hamilton. Hamilton defined a quaternion as the <b>quotient</b> of two vectors (two lines in a three-dimensional space). Quaternions are generally represented in the form:


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;a&plus;b\&space;\mathbf&space;{i}&space;&plus;c\&space;\mathbf&space;{j}&space;&plus;d\&space;\mathbf&space;{k}&space;}" title="https://latex.codecogs.com/svg.image?{\displaystyle a+b\ \mathbf {i} +c\ \mathbf {j} +d\ \mathbf {k} }" />

where a, b, c, and d are real numbers; and i, j, and k are the basic quaternions ( symbols that can be interpreted as unit-vectors pointing along the three spatial axes).


a quaternion <img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;q=a&plus;b\,\mathbf&space;{i}&space;&plus;c\,\mathbf&space;{j}&space;&plus;d\,\mathbf&space;{k}&space;}" title="https://latex.codecogs.com/svg.image?{\displaystyle q=a+b\,\mathbf {i} +c\,\mathbf {j} +d\,\mathbf {k} }" />, as consisting of a scalar part and a vector part. 
The quaternion <img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;b\,\mathbf&space;{i}&space;&plus;c\,\mathbf&space;{j}&space;&plus;d\,\mathbf&space;{k}&space;}" title="https://latex.codecogs.com/svg.image?{\displaystyle b\,\mathbf {i} +c\,\mathbf {j} +d\,\mathbf {k} }" />  is called the vector part (sometimes imaginary part) of q, and <img src="https://latex.codecogs.com/svg.image?a" title="https://latex.codecogs.com/svg.image?a" /> is the scalar part (sometimes real part) of q.

## 4.1. Basis
The set of quaternions is made a 4-dimensional vector space over the real numbers, with <img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;\left\{1,\mathbf&space;{i}&space;,\mathbf&space;{j}&space;,\mathbf&space;{k}&space;\right\}}" title="https://latex.codecogs.com/svg.image?{\displaystyle \left\{1,\mathbf {i} ,\mathbf {j} ,\mathbf {k} \right\}}" /> as a basis, by the componentwise addition


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{aligned}\mathbf&space;{j\,k}&space;&=\mathbf&space;{i}&space;\,,\quad&space;&\mathbf&space;{k\,j}&space;&=-\mathbf&space;{i}&space;\,,\\\mathbf&space;{k\,i}&space;&=\mathbf&space;{j}&space;\,,\quad&space;&\mathbf&space;{i\,k}&space;&=-\mathbf&space;{j}&space;\,,\\\mathbf&space;{i\,j\,k}&space;&=-1\,,\quad&space;&\mathbf&space;{k}&space;^{2}&=-1\,\\\end{aligned}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{aligned}\mathbf {j\,k} &=\mathbf {i} \,,\quad &\mathbf {k\,j} &=-\mathbf {i} \,,\\\mathbf {k\,i} &=\mathbf {j} \,,\quad &\mathbf {i\,k} &=-\mathbf {j} \,,\\\mathbf {i\,j\,k} &=-1\,,\quad &\mathbf {k} ^{2}&=-1\,\\\end{aligned}}}" />



vector definition of a quaternion:

<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bq%7D%5Ctriangleq%20%5Cbegin%7Bbmatrix%7Dq_w%20%5C%5C%20%5Cmathbf%7Bq%7D_v%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7Dq_w%20%5C%5C%20q_x%20%5C%5C%20q_y%20%5C%5C%20q_z%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{q}\triangleq 
\begin{bmatrix}q_w \\ \mathbf{q}_v\end{bmatrix} =
\begin{bmatrix}q_w \\ q_x \\ q_y \\ q_z\end{bmatrix}" />



<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;\mathbf&space;{i}&space;^{2}=\mathbf&space;{j}&space;^{2}=\mathbf&space;{k}&space;^{2}=\mathbf&space;{i\,j\,k}&space;=-1}" title="https://latex.codecogs.com/svg.image?{\displaystyle \mathbf {i} ^{2}=\mathbf {j} ^{2}=\mathbf {k} ^{2}=\mathbf {i\,j\,k} =-1}" />


## 4.2. Inverse of Quaternions

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;(a&plus;b\,\mathbf&space;{i}&space;&plus;c\,\mathbf&space;{j}&space;&plus;d\,\mathbf&space;{k}&space;)^{-1}={\frac&space;{1}{a^{2}&plus;b^{2}&plus;c^{2}&plus;d^{2}}}\,(a-b\,\mathbf&space;{i}&space;-c\,\mathbf&space;{j}&space;-d\,\mathbf&space;{k}&space;).}" title="https://latex.codecogs.com/svg.image?{\displaystyle (a+b\,\mathbf {i} +c\,\mathbf {j} +d\,\mathbf {k} )^{-1}={\frac {1}{a^{2}+b^{2}+c^{2}+d^{2}}}\,(a-b\,\mathbf {i} -c\,\mathbf {j} -d\,\mathbf {k} ).}" />








## 4.3. Quaternions Multiplication (Hamilton product)

For two elements <img src="https://latex.codecogs.com/svg.image?a_1&space;&plus;&space;b_1i&space;&plus;c_1j&plus;d_1k" title="https://latex.codecogs.com/svg.image?a_1 + b_1i +c_1j+d_1k" /> and <img src="https://latex.codecogs.com/svg.image?a_2&space;&plus;&space;b_2i&space;&plus;c_2j&plus;d_2k" title="https://latex.codecogs.com/svg.image?a_2 + b_2i +c_2j+d_2k" />, their product, called the Hamilton product and is determined by distributive law:

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?(a_1&space;&plus;&space;b_1i&space;&plus;c_1j&plus;d_1k)(a_2&space;&plus;&space;b_2i&space;&plus;c_2j&plus;d_2k)" title="https://latex.codecogs.com/svg.image?(a_1 + b_1i +c_1j+d_1k)(a_2 + b_2i +c_2j+d_2k)" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?=&space;{\begin{alignedat}{}&a_{1}a_{2}&&&plus;a_{1}b_{2}\mathbf&space;{i}&space;&&&plus;a_{1}c_{2}\mathbf&space;{j}&space;&&&plus;a_{1}d_{2}\mathbf&space;{k}&space;\\{}&plus;{}&b_{1}a_{2}\mathbf&space;{i}&space;&&&plus;b_{1}b_{2}\mathbf&space;{i}&space;^{2}&&&plus;b_{1}c_{2}\mathbf&space;{ij}&space;&&&plus;b_{1}d_{2}\mathbf&space;{ik}&space;\\{}&plus;{}&c_{1}a_{2}\mathbf&space;{j}&space;&&&plus;c_{1}b_{2}\mathbf&space;{ji}&space;&&&plus;c_{1}c_{2}\mathbf&space;{j}&space;^{2}&&&plus;c_{1}d_{2}\mathbf&space;{jk}&space;\\{}&plus;{}&d_{1}a_{2}\mathbf&space;{k}&space;&&&plus;d_{1}b_{2}\mathbf&space;{ki}&space;&&&plus;d_{1}c_{2}\mathbf&space;{kj}&space;&&&plus;d_{1}d_{2}\mathbf&space;{k}&space;^{2}\end{alignedat}}" title="https://latex.codecogs.com/svg.image?= {\begin{alignedat}{}&a_{1}a_{2}&&+a_{1}b_{2}\mathbf {i} &&+a_{1}c_{2}\mathbf {j} &&+a_{1}d_{2}\mathbf {k} \\{}+{}&b_{1}a_{2}\mathbf {i} &&+b_{1}b_{2}\mathbf {i} ^{2}&&+b_{1}c_{2}\mathbf {ij} &&+b_{1}d_{2}\mathbf {ik} \\{}+{}&c_{1}a_{2}\mathbf {j} &&+c_{1}b_{2}\mathbf {ji} &&+c_{1}c_{2}\mathbf {j} ^{2}&&+c_{1}d_{2}\mathbf {jk} \\{}+{}&d_{1}a_{2}\mathbf {k} &&+d_{1}b_{2}\mathbf {ki} &&+d_{1}c_{2}\mathbf {kj} &&+d_{1}d_{2}\mathbf {k} ^{2}\end{alignedat}}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?={\displaystyle&space;{\begin{alignedat}{}&a_{1}a_{2}&&-b_{1}b_{2}&&-c_{1}c_{2}&&-d_{1}d_{2}\\{}&plus;{}(&a_{1}b_{2}&&&plus;b_{1}a_{2}&&&plus;c_{1}d_{2}&&-d_{1}c_{2})\mathbf&space;{i}&space;\\{}&plus;{}(&a_{1}c_{2}&&-b_{1}d_{2}&&&plus;c_{1}a_{2}&&&plus;d_{1}b_{2})\mathbf&space;{j}&space;\\{}&plus;{}(&a_{1}d_{2}&&&plus;b_{1}c_{2}&&-c_{1}b_{2}&&&plus;d_{1}a_{2})\mathbf&space;{k}&space;\end{alignedat}}}" title="https://latex.codecogs.com/svg.image?={\displaystyle {\begin{alignedat}{}&a_{1}a_{2}&&-b_{1}b_{2}&&-c_{1}c_{2}&&-d_{1}d_{2}\\{}+{}(&a_{1}b_{2}&&+b_{1}a_{2}&&+c_{1}d_{2}&&-d_{1}c_{2})\mathbf {i} \\{}+{}(&a_{1}c_{2}&&-b_{1}d_{2}&&+c_{1}a_{2}&&+d_{1}b_{2})\mathbf {j} \\{}+{}(&a_{1}d_{2}&&+b_{1}c_{2}&&-c_{1}b_{2}&&+d_{1}a_{2})\mathbf {k} \end{alignedat}}}" />




<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bpq%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20p_w%20q_w%20-%20p_x%20q_x%20-%20p_y%20q_y%20-%20p_z%20q_z%20%5C%5C%20p_w%20q_x%20&plus;%20p_x%20q_w%20&plus;%20p_y%20q_z%20-%20p_z%20q_y%20%5C%5C%20p_w%20q_y%20-%20p_x%20q_z%20&plus;%20p_y%20q_w%20&plus;%20p_z%20q_x%20%5C%5C%20p_w%20q_z%20&plus;%20p_x%20q_y%20-%20p_y%20q_x%20&plus;%20p_z%20q_w%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{pq} =
\begin{bmatrix}
    p_w q_w - p_x q_x - p_y q_y - p_z q_z \\
    p_w q_x + p_x q_w + p_y q_z - p_z q_y \\
    p_w q_y - p_x q_z + p_y q_w + p_z q_x \\
    p_w q_z + p_x q_y - p_y q_x + p_z q_w
\end{bmatrix}" />



## 4.4. Quaternion as Orientation

Any orientation in a three-dimensional euclidean space of a frame <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />  with respect to a frame <img src="https://latex.codecogs.com/svg.latex?S" alt="https://latex.codecogs.com/svg.latex?S" />  can be represented by a unit quaternion (a.k.a. versor), <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bq%7D%5Cin%5Cmathbb%7BH%7D%5E4" alt="https://latex.codecogs.com/svg.latex?\mathbf{q}\in\mathbb{H}^4" /> , in Hamiltonian space defined as:


<img src="https://latex.codecogs.com/svg.latex?%5ES_B%5Cmathbf%7Bq%7D%20%3D%20%5Cbegin%7Bbmatrix%7Dq_w%5C%5Cq_x%5C%5Cq_y%5C%5Cq_z%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20%5Ccos%5Cfrac%7B%5Calpha%7D%7B2%7D%5C%5Ce_x%5Csin%5Cfrac%7B%5Calpha%7D%7B2%7D%5C%5Ce_y%5Csin%5Cfrac%7B%5Calpha%7D%7B2%7D%5C%5Ce_z%5Csin%5Cfrac%7B%5Calpha%7D%7B2%7D%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?^S_B\mathbf{q} = \begin{bmatrix}q_w\\q_x\\q_y\\q_z\end{bmatrix} =
\begin{bmatrix}
\cos\frac{\alpha}{2}\\e_x\sin\frac{\alpha}{2}\\e_y\sin\frac{\alpha}{2}\\e_z\sin\frac{\alpha}{2}
\end{bmatrix}" />




where <img src="https://latex.codecogs.com/svg.latex?\alpha" alt="https://latex.codecogs.com/svg.latex?\alpha" />  is the rotation angle and e  is the unit vector representing the rotation axis.



The orientation of frame <img src="https://latex.codecogs.com/svg.latex?S" alt="https://latex.codecogs.com/svg.latex?S" />
 relative to frame <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" /> is  the conjugate quaternion:

<img src="https://latex.codecogs.com/svg.latex?%5ES_B%5Cmathbf%7Bq%7D%5E*%20%3D%20%5C%2C%5EB_S%5Cmathbf%7Bq%7D%20%3D%20%5Cbegin%7Bbmatrix%7Dq_w%5C%5C-q_x%5C%5C-q_y%5C%5C-q_z%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?^S_B\mathbf{q}^* = \,^B_S\mathbf{q} = \begin{bmatrix}q_w\\-q_x\\-q_y\\-q_z\end{bmatrix}" />


the sequence of rotations follows the subscript cancellation rule:


<img src="https://latex.codecogs.com/svg.latex?%5EC_A%5Cmathbf%7Bq%7D%20%3D%20%5C%2C%5EC_B%5Cmathbf%7Bq%7D%20%5C%2C%20%5EB_A%5Cmathbf%7Bq%7D" alt="https://latex.codecogs.com/svg.latex?^C_A\mathbf{q} = \,^C_B\mathbf{q} \, ^B_A\mathbf{q}" />


## 4.5 Changing Frame of Reference with Unit Quaternion
If you have a vector that has been expressed in frame A:
<br/>
<img src="https://latex.codecogs.com/svg.latex?%5EA%5Cmathbf%7Bv%7D%3D%5Cbegin%7Bbmatrix%7Dv_x%20%5C%5C%20v_y%20%5C%5C%20v_z%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?^A\mathbf{v}=\begin{bmatrix}v_x \\ v_y \\ v_z\end{bmatrix}" />

<br/>
If you want to express it in frame B, First express it as : 

<br/>
<img src="https://latex.codecogs.com/svg.latex?%5EA%5Cmathbf%7Bv%7D_q%3D%5Cbegin%7Bbmatrix%7D0%5C%5Cv_x%20%5C%5C%20v_y%20%5C%5C%20v_z%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?^A\mathbf{v}_q=\begin{bmatrix}0\\v_x \\ v_y \\ v_z\end{bmatrix}" />


<br/>
<img src="https://latex.codecogs.com/svg.latex?%5EB%5Cmathbf%7Bv%7D_q%20%3D%20%5C%2C%20%5EB_A%5Cmathbf%7Bq%7D%20%5C%2C%20%5EA%5Cmathbf%7Bv%7D_q%20%5C%2C%20%5EB_A%5Cmathbf%7Bq%7D%5E*" alt="https://latex.codecogs.com/svg.latex?^B\mathbf{v}_q = \, ^B_A\mathbf{q} \, ^A\mathbf{v}_q \, ^B_A\mathbf{q}^*" />


The inverse rotation:


<img src="https://latex.codecogs.com/svg.latex?%5EA%5Cmathbf%7Bv%7D_q%20%3D%20%5C%2C%20%5EB_A%5Cmathbf%7Bq%7D%5E*%20%5C%2C%20%5EB%5Cmathbf%7Bv%7D_q%20%5C%2C%20%5EB_A%5Cmathbf%7Bq%7D%20%3D%20%5C%2C%20%5EA_B%5Cmathbf%7Bq%7D%20%5C%2C%20%5EB%5Cmathbf%7Bv%7D_q%20%5C%2C%20%5EA_B%5Cmathbf%7Bq%7D%5E*"  alt="https://latex.codecogs.com/svg.latex?^A\mathbf{v}_q = \, ^B_A\mathbf{q}^* \, ^B\mathbf{v}_q \, ^B_A\mathbf{q} = \, ^A_B\mathbf{q} \, ^B\mathbf{v}_q \, ^A_B\mathbf{q}^*" />



These rotations can also be expressed Direction Cosine Matrix:


<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BR%7D%28%5EB_A%5Cmathbf%7Bq%7D%29%20%3D%20%5Cbegin%7Bbmatrix%7D%20q_w%5E2&plus;q_x%5E2-q_y%5E2-q_z%5E2%20%26%202%28q_xq_y%20-%20q_wq_z%29%20%26%202%28q_xq_z%20&plus;%20q_wq_y%29%20%5C%5C%202%28q_xq_y%20&plus;%20q_wq_z%29%20%26%20q_w%5E2-q_x%5E2&plus;q_y%5E2-q_z%5E2%20%26%202%28q_yq_z%20-%20q_wq_x%29%20%5C%5C%202%28q_xq_z%20-%20q_wq_y%29%20%26%202%28q_wq_x%20&plus;%20q_yq_z%29%20%26%20q_w%5E2-q_x%5E2-q_y%5E2&plus;q_z%5E2%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{R}(^B_A\mathbf{q}) =
\begin{bmatrix} q_w^2+q_x^2-q_y^2-q_z^2 & 2(q_xq_y - q_wq_z) & 2(q_xq_z + q_wq_y) \\ 2(q_xq_y + q_wq_z) & q_w^2-q_x^2+q_y^2-q_z^2 & 2(q_yq_z - q_wq_x) \\ 2(q_xq_z - q_wq_y) & 2(q_wq_x + q_yq_z) & q_w^2-q_x^2-q_y^2+q_z^2 \end{bmatrix}" />




## 4.6. Conversion between quaternions and Euler angles



A unit quaternion can be described as:

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;\mathbf&space;{q}&space;={\begin{bmatrix}q_{0}&q_{1}&q_{2}&q_{3}\end{bmatrix}}^{T}={\begin{bmatrix}q_{w}&q_{x}&q_{y}&q_{z}\end{bmatrix}}^{T}}" title="https://latex.codecogs.com/svg.image?{\displaystyle \mathbf {q} ={\begin{bmatrix}q_{0}&q_{1}&q_{2}&q_{3}\end{bmatrix}}^{T}={\begin{bmatrix}q_{w}&q_{x}&q_{y}&q_{z}\end{bmatrix}}^{T}}" />

where 

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;|\mathbf&space;{q}&space;|^{2}=q_{0}^{2}&plus;q_{1}^{2}&plus;q_{2}^{2}&plus;q_{3}^{2}=q_{w}^{2}&plus;q_{x}^{2}&plus;q_{y}^{2}&plus;q_{z}^{2}=1}" title="https://latex.codecogs.com/svg.image?{\displaystyle |\mathbf {q} |^{2}=q_{0}^{2}+q_{1}^{2}+q_{2}^{2}+q_{3}^{2}=q_{w}^{2}+q_{x}^{2}+q_{y}^{2}+q_{z}^{2}=1}" />


To get the rotation matrix:


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;R={\begin{bmatrix}1-2(q_{2}^{2}&plus;q_{3}^{2})&2(q_{1}q_{2}-q_{0}q_{3})&2(q_{0}q_{2}&plus;q_{1}q_{3})\\2(q_{1}q_{2}&plus;q_{0}q_{3})&1-2(q_{1}^{2}&plus;q_{3}^{2})&2(q_{2}q_{3}-q_{0}q_{1})\\2(q_{1}q_{3}-q_{0}q_{2})&2(q_{0}q_{1}&plus;q_{2}q_{3})&1-2(q_{1}^{2}&plus;q_{2}^{2})\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle R={\begin{bmatrix}1-2(q_{2}^{2}+q_{3}^{2})&2(q_{1}q_{2}-q_{0}q_{3})&2(q_{0}q_{2}+q_{1}q_{3})\\2(q_{1}q_{2}+q_{0}q_{3})&1-2(q_{1}^{2}+q_{3}^{2})&2(q_{2}q_{3}-q_{0}q_{1})\\2(q_{1}q_{3}-q_{0}q_{2})&2(q_{0}q_{1}+q_{2}q_{3})&1-2(q_{1}^{2}+q_{2}^{2})\end{bmatrix}}}" />

To get the roll pitch, yaw:

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}\phi&space;\\\theta&space;\\\psi&space;\end{bmatrix}}={\begin{bmatrix}{\mbox{atan2}}\left(2(q_{0}q_{1}&plus;q_{2}q_{3}),1-2(q_{1}^{2}&plus;q_{2}^{2})\right)\\-\pi&space;/2&plus;2\,{\mbox{atan2}}\left({\sqrt&space;{1&plus;2(q_{0}q_{2}-q_{1}q_{3})}},{\sqrt&space;{1-2(q_{0}q_{2}-q_{1}q_{3})}}\right)\\{\mbox{atan2}}\left(2(q_{0}q_{3}&plus;q_{1}q_{2}),1-2(q_{2}^{2}&plus;q_{3}^{2})\right)\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}\phi \\\theta \\\psi \end{bmatrix}}={\begin{bmatrix}{\mbox{atan2}}\left(2(q_{0}q_{1}+q_{2}q_{3}),1-2(q_{1}^{2}+q_{2}^{2})\right)\\-\pi /2+2\,{\mbox{atan2}}\left({\sqrt {1+2(q_{0}q_{2}-q_{1}q_{3})}},{\sqrt {1-2(q_{0}q_{2}-q_{1}q_{3})}}\right)\\{\mbox{atan2}}\left(2(q_{0}q_{3}+q_{1}q_{2}),1-2(q_{2}^{2}+q_{3}^{2})\right)\end{bmatrix}}}" />



A very good article to read about [quaternions](https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html)


### Quaternion Conventions: Hamilton and JPL


Refs: [1](https://fzheng.me/2017/11/12/quaternion_conventions_en/)


## 4.7. Quaternion Representing the Rotation From One Vector to Another


Refs: [1](https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another)

## 4.8. Quaternions and  Axis-Angle Representation 
Quaternions can encode axis-angle representation in four numbers and can be used to apply the corresponding rotation to a position vector <img src="https://latex.codecogs.com/svg.image?(x,y,z)" alt="https://latex.codecogs.com/svg.image?(x,y,z)" />, representing a point relative to the origin in <img src="https://latex.codecogs.com/svg.image?\mathbb{R}^3" alt="https://latex.codecogs.com/svg.image?\mathbb{R}^3"/>.


Euclidean vectors such as <img src="https://latex.codecogs.com/svg.image?(2,3,4)" alt="https://latex.codecogs.com/svg.image?(2,3,4)" /> or <img src="https://latex.codecogs.com/svg.image?(a_x,a_y,a_z)" alt="https://latex.codecogs.com/svg.image?(a_x,a_y,a_z)" /> can be rewritten as <img src="https://latex.codecogs.com/svg.image?2i+3j+4k" alt="https://latex.codecogs.com/svg.image?2i+3j+4k" /> or  <img src="https://latex.codecogs.com/svg.image?(a_xi,a_yj,a_zk)"  alt="https://latex.codecogs.com/svg.image?(a_xi,a_yj,a_zk)" /> , where i, j, k are unit vectors representing the three Cartesian axes (traditionally x, y, z), and also obey the multiplication rules of the fundamental quaternion units.


Therefore, a rotation of angle <img src="https://latex.codecogs.com/svg.image?\theta" alt="https://latex.codecogs.com/svg.image?\theta" />  around the axis defined by the unit vector <img src="https://latex.codecogs.com/svg.image?{\vec%20{u}}=(u_{x},u_{y},u_{z})=u_{x}\mathbf%20{i}%20+u_{y}\mathbf%20{j}%20+u_{z}\mathbf%20{k}" alt="https://latex.codecogs.com/svg.image?{\vec {u}}=(u_{x},u_{y},u_{z})=u_{x}\mathbf {i} +u_{y}\mathbf {j} +u_{z}\mathbf {k}" />


can be represented by a quaternion using an extension of Euler's formula:

<img src="https://latex.codecogs.com/svg.image?\mathbf%20{q}%20=e^{{\frac%20{\theta%20}{2}}{(u_{x}\mathbf%20{i}%20+u_{y}\mathbf%20{j}%20+u_{z}\mathbf%20{k}%20)}}=\cos%20{\frac%20{\theta%20}{2}}+(u_{x}\mathbf%20{i}%20+u_{y}\mathbf%20{j}%20+u_{z}\mathbf%20{k}%20)\sin%20{\frac%20{\theta%20}{2}}" alt="https://latex.codecogs.com/svg.image?\mathbf {q} =e^{{\frac {\theta }{2}}{(u_{x}\mathbf {i} +u_{y}\mathbf {j} +u_{z}\mathbf {k} )}}=\cos {\frac {\theta }{2}}+(u_{x}\mathbf {i} +u_{y}\mathbf {j} +u_{z}\mathbf {k} )\sin {\frac {\theta }{2}}" />



The desired rotation can be applied to an ordinary vector 

<img src="https://latex.codecogs.com/svg.image?\mathbf%20{p}%20=(p_{x},p_{y},p_{z})=p_{x}\mathbf%20{i}%20+p_{y}\mathbf%20{j}%20+p_{z}\mathbf%20{k}" alt="https://latex.codecogs.com/svg.image?\mathbf {p} =(p_{x},p_{y},p_{z})=p_{x}\mathbf {i} +p_{y}\mathbf {j} +p_{z}\mathbf {k}" />  in 3-dimensional space, considered as a quaternion with a real coordinate equal to zero, by the followings:




<img src="https://latex.codecogs.com/svg.image?\mathbf%20{p%27}%20=\mathbf%20{q}%20\mathbf%20{p}%20\mathbf%20{q}%20^{-1}" alt="https://latex.codecogs.com/svg.image?\mathbf {p'} =\mathbf {q} \mathbf {p} \mathbf {q} ^{-1}" />


In this instance, q is a unit quaternion and

<img src="https://latex.codecogs.com/svg.image?\mathbf%20{q}%20^{-1}=e^{-{\frac%20{\theta%20}{2}}{(u_{x}\mathbf%20{i}%20+u_{y}\mathbf%20{j}%20+u_{z}\mathbf%20{k}%20)}}=\cos%20{\frac%20{\theta%20}{2}}-(u_{x}\mathbf%20{i}%20+u_{y}\mathbf%20{j}%20+u_{z}\mathbf%20{k}%20)\sin%20{\frac%20{\theta%20}{2}}" alt="https://latex.codecogs.com/svg.image?\mathbf {q} ^{-1}=e^{-{\frac {\theta }{2}}{(u_{x}\mathbf {i} +u_{y}\mathbf {j} +u_{z}\mathbf {k} )}}=\cos {\frac {\theta }{2}}-(u_{x}\mathbf {i} +u_{y}\mathbf {j} +u_{z}\mathbf {k} )\sin {\frac {\theta }{2}}." />

Example: rotate the point vector (1,0,0) around y axis (0,1,0)  90 degrees.

```cpp
// P  = [0, p1, p2, p3]  <-- point vector
// alpha = angle to rotate
//[x, y, z] = axis to rotate around (unit vector)
// R = [cos(alpha/2), sin(alpha/2)*x, sin(alpha/2)*y, sin(alpha/2)*z] <-- rotation
// R' = [w, -x, -y, -z]
// P' = RPR'
// P' = H(H(R, P), R')

Eigen::Vector3d p(1, 0, 0);

Quaternion P;
P.w = 0;
P.x = p(0);
P.y = p(1);
P.z = p(2);

// rotation of 90 degrees about the y-axis
double alpha = M_PI / 2;
Quaternion R;
Eigen::Vector3d r(0, 1, 0);
r = r.normalized();


R.w = cos(alpha / 2);
R.x = sin(alpha / 2) * r(0);
R.y = sin(alpha / 2) * r(1);
R.z = sin(alpha / 2) * r(2);

std::cout << R.w << "," << R.x << "," << R.y << "," << R.z << std::endl;

Quaternion R_prime = quaternionInversion(R);
Quaternion P_prime = quaternionMultiplication(quaternionMultiplication(R, P), R_prime);

/*rotation of 90 degrees about the y-axis for the point (1, 0, 0). The result
is (0, 0, -1). (Note that the first element of P' will always be 0 and can
therefore be discarded.)
*/

```

Refs: [1](https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion)  

let's demonstrate the rotation of a vector using both the quaternion and the axis-angle methods, using the same angle <img src="https://latex.codecogs.com/svg.latex?%5Ctheta" alt="\theta" /> and a unit axis vector <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bu%7D" alt="\mathbf{u}" /> .

Consider:
- Vector <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%20%3D%20%5Ba_x%2C%20b_y%2C%20c_z%5D" alt="\mathbf{v} = [a_x, b_y, c_z]" />
- Rotation axis <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bu%7D%20%3D%20%5Bu_x%2C%20u_y%2C%20u_z%5D" alt="\mathbf{u} = [u_x, u_y, u_z]" /> (assuming <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bu%7D" alt="\mathbf{u}" /> is a unit vector)
- Rotation angle <img src="https://latex.codecogs.com/svg.latex?%5Ctheta" alt="\theta" />



##  Fully Represent a Frame With Quaternions

To represent a position in 3D space use a combination of a quaternion for orientation and a vector for the position.

- For the position, you can use a `Vector3d`, which is a vector of three doubles.
- For the orientation, use a `Quaterniond`, which is a quaternion that uses double precision.

** Define Position and Orientation **
```cpp
Eigen::Vector3d position(1.0, 2.0, 3.0); // Example position (x, y, z)
Eigen::Quaterniond orientation; // Quaternion for orientation
```

**Initialize the Quaternion**:
- You can initialize the quaternion in several ways, such as from an axis-angle representation, from a rotation matrix, or directly setting its components.

```cpp
// Example: initializing the quaternion from an axis and an angle
Eigen::Vector3d axis(0, 1, 0); // Rotation around the y-axis
double angle = M_PI / 4; // Rotate 45 degrees
orientation = Eigen::AngleAxisd(angle, axis.normalized());
```

**Using the Position and Orientation**:
- Once you have the position and the quaternion, you can use them to transform points, calculate rotations, etc.

```cpp
// Example: rotating a point using the quaternion
Eigen::Vector3d point(1, 0, 0);
Eigen::Vector3d rotatedPoint = orientation * point;
```

**Combining Position and Orientation**:
- If you want to create a transformation matrix that includes both the position and orientation, you can do so using an affine transformation.

```cpp
Eigen::Affine3d transform = Eigen::Translation3d(position) * orientation;
```

## Multiplication of Frames Expressed with Quaternions
Here's a complete example putting it all together:

```cpp

double x1 = 1.0, y1 = 0.0, z1 = 0.0;
double q_w1 = 1.0, q_x1 = 0.0, q_y1 = 0.0, q_z1 = 0.0;
double x2 = 1.0, y2 = 0.0, z2 = 0.0;
double q_w2 = 1.0, q_x2 = 0.0, q_y2 = 0.0, q_z2 = 0.0;

Eigen::Affine3d pose1 = Eigen::Translation3d(x1, y1, z1) * Eigen::Quaterniond(q_w1, q_x1, q_y1, q_z1);
Eigen::Affine3d pose2 = Eigen::Translation3d(x2, y2, z2) * Eigen::Quaterniond(q_w2, q_x2, q_y2, q_z2);

Eigen::Affine3d result = pose1 * pose2;

Eigen::Vector3d res_translation = result.translation();
Eigen::Quaterniond res_quaternion(result.rotation());

std::cout << "Resulting Pose Translation: " << res_translation.transpose() << std::endl;
std::cout << "Resulting Pose Quaternion: " 
      << res_quaternion.w() << " " 
      << res_quaternion.x() << " " 
      << res_quaternion.y() << " " 
      << res_quaternion.z() << std::endl;
```

## 1. Rotating using Quaternion:

First, convert the axis-angle representation to a quaternion:

<img src="https://latex.codecogs.com/svg.latex?q%20%3D%20%5Ccos%5Cleft%28%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Cright%29%20&plus;%20%5Csin%5Cleft%28%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Cright%29%28u_x%5Cmathbf%7Bi%7D%20&plus;%20u_y%5Cmathbf%7Bj%7D%20&plus;%20u_z%5Cmathbf%7Bk%7D%29" alt="q = \cos\left(\frac{\theta}{2}\right) + \sin\left(\frac{\theta}{2}\right)(u_x\mathbf{i} + u_y\mathbf{j} + u_z\mathbf{k}) " />


Now, to rotate the vector:
<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%27%7D%20%3D%20q%20%5Ctimes%20%5Cmathbf%7Bv%7D%20%5Ctimes%20q%5E*" alt="\mathbf{v'} = q \times \mathbf{v} \times q^*" />
Where:
<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%20%3D%200%20&plus;%20a_x%5Cmathbf%7Bi%7D%20&plus;%20b_y%5Cmathbf%7Bj%7D%20&plus;%20c_z%5Cmathbf%7Bk%7D" alt="\mathbf{v} = 0 + a_x\mathbf{i} + b_y\mathbf{j} + c_z\mathbf{k}" />


And <img src="https://latex.codecogs.com/svg.latex?q%5E*" alt="q^*" /> is the conjugate of <img src="https://latex.codecogs.com/svg.latex?q" alt="q" />.

## 2. Rotating using Axis-Angle:

<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%27%7D%20%3D%20%5Cmathbf%7Bv%7D%20%5Ccos%28%5Ctheta%29%20&plus;%20%28%5Cmathbf%7Bu%7D%20%5Ctimes%20%5Cmathbf%7Bv%7D%29%20%5Csin%28%5Ctheta%29%20&plus;%20%5Cmathbf%7Bu%7D%20%28%5Cmathbf%7Bu%7D%20%5Ccdot%20%5Cmathbf%7Bv%7D%29%20%281%20-%20%5Ccos%28%5Ctheta%29%29" alt="\mathbf{v'} = \mathbf{v} \cos(\theta) + (\mathbf{u} \times \mathbf{v}) \sin(\theta) + \mathbf{u} (\mathbf{u} \cdot \mathbf{v}) (1 - \cos(\theta))" />



---

**Example**:


<img src="" alt="" />










Let's rotate the vector <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%20%3D%20%5B1%2C%200%2C%200%5D" alt="\mathbf{v} = [1, 0, 0]" /> by <img src="https://latex.codecogs.com/svg.latex?%5Ctheta%20%3D%20%5Cfrac%7B%5Cpi%7D%7B2%7D" alt="\theta = \frac{\pi}{2}" /> (90 degrees) around the unit axis <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bu%7D%20%3D%20%5B0%2C%200%2C%201%5D" alt="\mathbf{u} = [0, 0, 1]" />:

Using the Quaternion method:
1. Convert to quaternion: <img src="https://latex.codecogs.com/svg.latex?q%20%3D%20%5Ccos%2845%5E%5Ccirc%29%20&plus;%200%5Cmathbf%7Bi%7D%20&plus;%200%5Cmathbf%7Bj%7D%20&plus;%20%5Csin%2845%5E%5Ccirc%29%5Cmathbf%7Bk%7D" alt="q = \cos(45^\circ) + 0\mathbf{i} + 0\mathbf{j} + \sin(45^\circ)\mathbf{k}" />
2. Rotate: <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%27%7D%20%3D%20q%20%5Ctimes%20%5B0%2C%201%2C%200%2C%200%5D%20%5Ctimes%20q%5E*" alt="\mathbf{v'} = q \times [0, 1, 0, 0] \times q^*" />
   Result: 
   
   



Using the Axis-Angle method:
1. Calculate: <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%27%7D%20%3D%20%5B1%2C%200%2C%200%5D%20%5Ccos%2890%5E%5Ccirc%29%20&plus;%20%5B0%2C%201%2C%200%5D%20%5Csin%2890%5E%5Ccirc%29%20&plus;%20%5B0%2C%200%2C%201%5D%20%28%5B0%2C%200%2C%201%5D%20%5Ccdot%20%5B1%2C%200%2C%200%5D%29%20%281%20-%20%5Ccos%2890%5E%5Ccirc%29%29" alt="\mathbf{v'} = [1, 0, 0] \cos(90^\circ) + [0, 1, 0] \sin(90^\circ) + [0, 0, 1] ([0, 0, 1] \cdot [1, 0, 0]) (1 - \cos(90^\circ))" />   

Result: <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%27%7D%20%3D%20%5B0%2C%201%2C%200%5D" alt="\mathbf{v'} = [0, 1, 0]" />

In both methods, the result is <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%27%7D%20%3D%20%5B0%2C%201%2C%200%5D" alt="\mathbf{v'} = [0, 1, 0]" />, which is a 90-degree rotation of the original vector around the z-axis.

## Rotating a vector using a quaternion

how to rotate a vector <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D" alt="\mathbf{v}" /> by a quaternion <img src="https://latex.codecogs.com/svg.latex?q" alt="q" />:

1. **Represent the Vector as a Quaternion**:
If your vector is <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%20%3D%20%5Bv_x%2C%20v_y%2C%20v_z%5D" alt="\mathbf{v} = [v_x, v_y, v_z]" />, represent it as a quaternion:
<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%20%3D%200%20&plus;%20v_x%5Cmathbf%7Bi%7D%20&plus;%20v_y%5Cmathbf%7Bj%7D%20&plus;%20v_z%5Cmathbf%7Bk%7D" alt="\mathbf{v} = 0 + v_x\mathbf{i} + v_y\mathbf{j} + v_z\mathbf{k} " />

2. **Quaternion Rotation**:
To rotate the vector by quaternion <img src="https://latex.codecogs.com/svg.latex?q" alt="https://latex.codecogs.com/svg.latex?q" />, use the following formula:
<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D_%7B%5Ctext%7Brot%7D%7D%20%3D%20q%20%5Ctimes%20%5Cmathbf%7Bv%7D%20%5Ctimes%20q%5E*" alt="\mathbf{v}_{\text{rot}} = q \times \mathbf{v} \times q^*" />
where <img src="https://latex.codecogs.com/svg.latex?q%5E*" alt="q^*" /> is the conjugate of <img src="https://latex.codecogs.com/svg.latex?q" alt="https://latex.codecogs.com/svg.latex?q" />.

3. **Extract the Rotated Vector**:
After the multiplication, your rotated vector is the imaginary part of the resulting quaternion.

**Example**:
Let's say you have a vector <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%20%3D%20%5B1%2C%200%2C%200%5D" alt="\mathbf{v} = [1, 0, 0]" /> and you want to rotate it by 90 degrees around the z-axis. The corresponding quaternion for this rotation is:
<img src="https://latex.codecogs.com/svg.latex?q%20%3D%20%5Ccos%28%5Ctheta/2%29%20&plus;%20%5Csin%28%5Ctheta/2%29%20%5Ctimes%20%5Cmathbf%7Baxis%7D%20%3D%20%5Ccos%2845%5E%5Ccirc%29%20&plus;%20%5Csin%2845%5E%5Ccirc%29k%20%3D%20%5Csqrt%7B2%7D/2%20&plus;%20%5Csqrt%7B2%7D/2k" alt="q = \cos(\theta/2) + \sin(\theta/2) \times \mathbf{axis} = \cos(45^\circ) + \sin(45^\circ)k = \sqrt{2}/2 + \sqrt{2}/2k" />


To rotate the vector:

1. Represent the vector as a quaternion:  <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%20%3D%200%20&plus;%201%5Cmathbf%7Bi%7D%20&plus;%200%5Cmathbf%7Bj%7D%20&plus;%200%5Cmathbf%7Bk%7D" alt="\mathbf{v} = 0 + 1\mathbf{i} + 0\mathbf{j} + 0\mathbf{k}" />  
2. Multiply: 
<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D_%7B%5Ctext%7Brot%7D%7D%20%3D%20q%20%5Ctimes%20%5Cmathbf%7Bv%7D%20%5Ctimes%20q%5E*" alt="\mathbf{v}_{\text{rot}} = q \times \mathbf{v} \times q^*" />

3. The imaginary part of <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D_%7B%5Ctext%7Brot%7D%7D" alt="\mathbf{v}_{\text{rot}}" /> is your rotated vector.

Using the above method, the vector `[1, 0, 0]` would be rotated to approximately `[0, 1, 0]` (assuming unit quaternions).

It's worth noting that using quaternions to represent and perform rotations can help avoid issues like gimbal lock, which can occur with Euler angles. Quaternions provide a compact and efficient way to represent 3D orientations and perform rotations.

## Transform a full representation of position (orientation and translation ) with quaternions


When you have a full representation of position using both orientation (rotation) and translation, and you want to transform it using quaternions, you'll need to consider both the rotational and translational components.

Let's denote:

- The source frame as:
  - Orientation (rotation) quaternion: <img src="https://latex.codecogs.com/svg.latex?q_s" alt="q_s" />
  - Translation vector: <img src="https://latex.codecogs.com/svg.latex?t_s" alt="t_s" />

- The transformation frame as:
  - Orientation (rotation) quaternion: <img src="https://latex.codecogs.com/svg.latex?q_t" alt="q_t" />
  - Translation vector: <img src="https://latex.codecogs.com/svg.latex?t_t" alt="t_t" />

To transform the source frame by the transformation frame:

1. Rotate the orientation of the source frame using the orientation of the transformation frame.
2. Rotate the translation of the source frame by the orientation of the transformation frame, then add the translation of the transformation frame.


a source frame represented by the orientation <img src="https://latex.codecogs.com/svg.latex?q_s" alt="q_s" /> and translation <img src="https://latex.codecogs.com/svg.latex?t_s" alt="t_s" /> using a transformation frame 
with orientation <img src="https://latex.codecogs.com/svg.latex?q_t" alt="q_t" /> and translation  <img src="https://latex.codecogs.com/svg.latex?t_t" alt="t_t" />, here's the mathematical breakdown:

1. **Compound the Rotations**:
   The resulting orientation <img src="https://latex.codecogs.com/svg.latex?q_%7Bcombined%7D" alt="q_{combined}" /> of the transformed source frame is found by quaternion multiplication:
   <img src="https://latex.codecogs.com/svg.latex?q_%7Bcombined%7D%20%3D%20q_t%20%5Ctimes%20q_s" alt="q_{combined} = q_t \times q_s" />

2. **Rotate the Source Translation and Add Transformation Translation**:
   
   First, convert the translation vector <img src="https://latex.codecogs.com/svg.latex?t_s" alt="t_s" /> of the source frame into a quaternion <img src="https://latex.codecogs.com/svg.latex?t_%7Bs%5C_quat%7D" alt="t_{s\_quat}" /> with a zero scalar part:
<img src="https://latex.codecogs.com/svg.latex?t_%7Bs%5C_quat%7D%20%3D%200%20&plus;%20t_%7Bs_x%7D%5Cmathbf%7Bi%7D%20&plus;%20t_%7Bs_y%7D%5Cmathbf%7Bj%7D%20&plus;%20t_%7Bs_z%7D%5Cmathbf%7Bk%7D" alt="t_{s\_quat} = 0 + t_{s_x}\mathbf{i} + t_{s_y}\mathbf{j} + t_{s_z}\mathbf{k}" />
   
   Then, rotate this quaternion using the orientation <img src="https://latex.codecogs.com/svg.latex?q_t" alt="q_t" /> of the transformation frame:
   <img src="https://latex.codecogs.com/svg.latex?t_%7Bs%5C_rotated%5C_quat%7D%20%3D%20q_t%20%5Ctimes%20t_%7Bs%5C_quat%7D%20%5Ctimes%20q_t%5E*" alt="t_{s\_rotated\_quat} = q_t \times t_{s\_quat} \times q_t^*" />
   where <img src="https://latex.codecogs.com/svg.latex?q_t%5E*" alt="q_t^*" /> is the conjugate of <img src="https://latex.codecogs.com/svg.latex?q_t" alt="q_t" />.

   The rotated translation vector <img src="https://latex.codecogs.com/svg.latex?t_%7Bs%5C_rotated%7D" alt="t_{s\_rotated}" /> is then the imaginary part (vector part) of <img src="https://latex.codecogs.com/svg.latex?t_%7Bs%5C_rotated%5C_quat%7D" alt="t_{s\_rotated\_quat}" />.

   Finally, add the translation vector <img src="https://latex.codecogs.com/svg.latex?t_t" alt="t_t" /> of the transformation frame to get the combined translation:
   <img src="https://latex.codecogs.com/svg.latex?t_%7Bcombined%7D%20%3D%20t_%7Bs%5C_rotated%7D%20&plus;%20t_t" alt="t_{combined} = t_{s\_rotated} + t_t" />

So, the final transformed source frame in the new reference frame is represented by orientation <img src="https://latex.codecogs.com/svg.latex?q_%7Bcombined%7D" alt="q_{combined}" /> and translation  <img src="https://latex.codecogs.com/svg.latex?t_%7Bcombined%7D" alt="t_{combined}" />.



Here's a Python code example using the `numpy` and `numpy-quaternion` libraries:

```python
import numpy as np
import quaternion

# Define quaternions and translations
# For the sake of the example, let's assume the following:
# A rotation of 45 degrees around the z-axis for both frames
# And a translation of (1,0,0) for both frames

angle = np.pi / 4
axis = np.array([0, 0, 1])

q_s = quaternion.from_rotation_vector(angle * axis)
t_s = np.array([1, 0, 0])

q_t = quaternion.from_rotation_vector(angle * axis)
t_t = np.array([1, 0, 0])

# 1. Compound the rotations
q_combined = q_t * q_s

# 2. Rotate the source translation and then translate
# Convert translation to quaternion
t_s_quat = np.quaternion(0, t_s[0], t_s[1], t_s[2])

# Rotate translation
t_s_rotated_quat = q_t * t_s_quat * q_t.inverse()

# Extract the vector part and add the transformation translation
t_combined = np.array([t_s_rotated_quat.x, t_s_rotated_quat.y, t_s_rotated_quat.z]) + t_t

print(f"Combined Orientation (Quaternion): {q_combined}")
print(f"Combined Translation: {t_combined}")
```


## Inverse of Full Pose (position and orientation ) expressed in Quaternions

If you have the pose of frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> expressed in the world frame as <img src="https://latex.codecogs.com/svg.latex?%5Bx%2C%20y%2C%20z%2C%20q1%2C%20q2%2C%20q3%2C%20q4%5D" alt="[x, y, z, q1, q2, q3, q4]" /> where <img src="https://latex.codecogs.com/svg.latex?%5Bx%2C%20y%2C%20z%5D" alt="[x, y, z]" /> is the position and <img src="https://latex.codecogs.com/svg.latex?%5Bq1%2C%20q2%2C%20q3%2C%20q4%5D" alt="[q1, q2, q3, q4]" /> is the quaternion representing the orientation, then you want to find the pose of the world frame with respect to frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />.

Given:
- Position of frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> in world frame: <img src="https://latex.codecogs.com/svg.latex?%5Bx%2C%20y%2C%20z%5D" alt="[x, y, z]" />
- Orientation of frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> in world frame (as quaternion): <img src="https://latex.codecogs.com/svg.latex?%5Bq1%2C%20q2%2C%20q3%2C%20q4%5D" alt="[q1, q2, q3, q4]" />

To compute the pose of the world in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />, we'll need to find the inverse transformation.

1. **Inverse Position**:
   The position of the world origin in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> coordinates is given by the negation of the original position:
   <img src="https://latex.codecogs.com/svg.latex?%5Bx%27%2C%20y%27%2C%20z%27%5D%20%3D%20%5B-x%2C%20-y%2C%20-z%5D" alt="[x', y', z'] = [-x, -y, -z]" />


2. **Inverse Orientation**:
   The orientation of the world frame with respect to frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> can be obtained by taking the conjugate of the given quaternion. The conjugate of a quaternion <img src="https://latex.codecogs.com/svg.latex?%5Bq1%2C%20q2%2C%20q3%2C%20q4%5D" alt="[q1, q2, q3, q4]" /> is given by:
   <img src="https://latex.codecogs.com/svg.latex?%5Bq1%27%2C%20q2%27%2C%20q3%27%2C%20q4%27%5D%20%3D%20%5Bq1%2C%20-q2%2C%20-q3%2C%20-q4%5D" alt="[q1', q2', q3', q4'] = [q1, -q2, -q3, -q4]" />

However, simply inverting the translation is not enough. The correct pose of the world in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> would require us to rotate the negated translation vector using the inverse orientation.

To do this, you'll express the negated position vector as a quaternion with zero scalar part: <img src="https://latex.codecogs.com/svg.latex?q_%7B%5Ctext%7Bpos%7D%7D%20%3D%20%5B0%2C%20-x%2C%20-y%2C%20-z%5D" alt=" q_{\text{pos}} = [0, -x, -y, -z] " />.

Then, you'll multiply this by the inverse orientation quaternion:
<img src="https://latex.codecogs.com/svg.latex?q_%7B%5Ctext%7Bresult%7D%7D%20%3D%20q_%7B%5Ctext%7Binv%7D%7D%20%5Ctimes%20q_%7B%5Ctext%7Bpos%7D%7D%20%5Ctimes%20q" alt=" q_{\text{result}} = q_{\text{inv}} \times q_{\text{pos}} \times q " />
where <img src="https://latex.codecogs.com/svg.latex?q" alt="q" /> is the original orientation quaternion, and <img src="https://latex.codecogs.com/svg.latex?q_%7B%5Ctext%7Binv%7D%7D" alt="q_{\text{inv}} " /> is its conjugate.

The resulting quaternion <img src="https://latex.codecogs.com/svg.latex?q_%7B%5Ctext%7Bresult%7D%7D" alt="q_{\text{result}}" /> will have its vector part (last three components) as the desired transformed position of the world in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />. The scalar part of <img src="https://latex.codecogs.com/svg.latex?q_%7B%5Ctext%7Bresult%7D%7D" alt="q_{\text{result}}" /> should be 0.

Finally:
- The position of the world in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> is the vector part of <img src="https://latex.codecogs.com/svg.latex?q_%7B%5Ctext%7Bresult%7D%7D" alt="q_{\text{result}}" />.
- The orientation of the world in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> is the conjugate of the given orientation: <img src="https://latex.codecogs.com/svg.latex?%5Bq1%27%2C%20q2%27%2C%20q3%27%2C%20q4%27%5D" alt="[q1', q2', q3', q4']" />.

## Example of relative pose of two camera and IMU
if the given transformations are the positions of the IMU expressed in the camera frames, then we need to slightly modify our approach.

Given:
- <img src="https://latex.codecogs.com/svg.latex?q_%7BC0-IMU%7D" alt="q_{C0-IMU}" />: Quaternion of `IMU `with respect to `Camera0`
- <img src="https://latex.codecogs.com/svg.latex?q_%7BC1-IMU%7D" alt="q_{C1-IMU}" />: Quaternion of `IMU` with respect to `Camera1`
- <img src="https://latex.codecogs.com/svg.latex?t_%7BC0-IMU%7D" alt="t_{C0-IMU}" />: Translation of `IMU` with respect to `Camera0`
- <img src="https://latex.codecogs.com/svg.latex?t_%7BC1-IMU%7D" alt="t_{C1-IMU}" />: Translation of `IMU` with respect to `Camera1`

We want to find:
- <img src="https://latex.codecogs.com/svg.latex?q_%7BC0-C1" alt="q_{C0-C1}" />: Quaternion of `Camera1` with respect to `Camera0`
- <img src="https://latex.codecogs.com/svg.latex?t_%7BC0-C1" alt="t_{C0-C1}" />: Translation of `Camera1` with respect to `Camera0`

The formulae are:
- <img src="https://latex.codecogs.com/svg.latex?q_%7BC0-C1%7D%20%3D%20q_%7BC0-IMU%7D%20%5Cotimes%20q_%7BC1-IMU%7D%5E%7B-1%7D" alt="q_{C0-C1} = q_{C0-IMU} \otimes q_{C1-IMU}^{-1}" />
- <img src="https://latex.codecogs.com/svg.latex?t_%7BC0-C1%7D%20%3D%20q_%7BC0-IMU%7D%20%5Cotimes%20%28t_%7BC1-IMU%7D%20-%20t_%7BC0-IMU%7D%29%20%5Cotimes%20q_%7BC0-IMU%7D%5E%7B-1%7D" alt="t_{C0-C1} = q_{C0-IMU} \otimes (t_{C1-IMU} - t_{C0-IMU}) \otimes q_{C0-IMU}^{-1}" />

Let's implement this in Python:

```python
import numpy as np
from pyquaternion import Quaternion

def relative_pose(q_C0_IMU, t_C0_IMU, q_C1_IMU, t_C1_IMU):
    # Calculate relative quaternion
    q_C0_C1 = q_C0_IMU * q_C1_IMU.inverse

    # Calculate relative translation
    t_diff = np.array(t_C1_IMU) - np.array(t_C0_IMU)
    t_C0_C1 = q_C0_IMU.rotate(t_diff)

    return q_C0_C1, t_C0_C1.tolist()

# Define quaternions and translations for IMU w.r.t Camera0 and Camera1
q_C0_IMU = Quaternion(w=0.6328142, x=0.3155095, y=-0.3155095, z=0.6328142)
t_C0_IMU = [0.234508, 0.028785, 0.039920]

q_C1_IMU = Quaternion(w=0.3155095, x=-0.6328142, y=-0.6328142, z=-0.3155095)
t_C1_IMU = [0.234508, 0.028785, -0.012908]

q_C0_C1, t_C0_C1 = relative_pose(q_C0_IMU, t_C0_IMU, q_C1_IMU, t_C1_IMU)
print("Quaternion of Camera1 w.r.t Camera0:", q_C0_C1)
print("Translation of Camera1 w.r.t Camera0:", t_C0_C1)
```

This Python code should give you the pose of Camera1 with respect to Camera0.




## Expressing Relative Pose using Quaternions  (subscript cancellation)

If Pose <img src="https://latex.codecogs.com/svg.latex?C" alt="C" />  express in Frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />  and pose of <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> expressed in <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />  using quaternions,  equation for finding the pose <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> expressed in <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />  using quaternions





1. **Rotations**:
Let's define the following quaternions for the rotations:
- <img src="https://latex.codecogs.com/svg.latex?Q%5E%7BA%7D_%7BB%7D" alt="Q^{A}_{B}" /> is the quaternion representing the rotation of frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> with respect to frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> (<img src="https://latex.codecogs.com/svg.latex?B" alt="B" />'s rotation expressed in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />).
- <img src="https://latex.codecogs.com/svg.latex?Q%5E%7BB%7D_%7BC%7D" alt="Q^{B}_{C}" /> is the quaternion representing the rotation of frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> with respect to frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> (<img src="https://latex.codecogs.com/svg.latex?C" alt="C" />'s rotation expressed in frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />).

The combined rotation of frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> with respect to frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />, <img src="https://latex.codecogs.com/svg.latex?Q%5E%7BA%7D_%7BC%7D" alt="Q^{A}_{C}" /> , is given by:
<img src="https://latex.codecogs.com/svg.latex?Q%5E%7BA%7D_%7BC%7D%20%3D%20Q%5E%7BA%7D_%7BB%7D%20%5Cotimes%20Q%5E%7BB%7D_%7BC%7D" alt="Q^{A}_{C} = Q^{A}_{B} \otimes Q^{B}_{C} " />



2. **Translations (positions)**:
If you have the following positions:
- <img src="https://latex.codecogs.com/svg.latex?P%5E%7BA%7D_%7BB%7D" alt="P^{A}_{B}" /> is the position of point <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> (or frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />'s origin) expressed in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />.
- <img src="https://latex.codecogs.com/svg.latex?P%5E%7BB%7D_%7BC%7D" alt="P^{B}_{C}" /> is the position of point <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> (or frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" />'s origin) expressed in frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />.

The position of point <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> (or frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" />'s origin) expressed in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />, <img src="https://latex.codecogs.com/svg.latex?P%5E%7BA%7D_%7BC%7D" alt="P^{A}_{C}" />, when considering rotations, is:
<img src="https://latex.codecogs.com/svg.latex?P%5E%7BA%7D_%7BC%7D%20%3D%20P%5E%7BA%7D_%7BB%7D%20&plus;%20Q%5E%7BA%7D_%7BB%7D%20%5Cotimes%20P%5E%7BB%7D_%7BC%7D%20%5Cotimes%20%28Q%5E%7BA%7D_%7BB%7D%29%5E%7B-1%7D" alt=" P^{A}_{C} = P^{A}_{B} + Q^{A}_{B} \otimes P^{B}_{C} \otimes (Q^{A}_{B})^{-1} " />


Where <img src="https://latex.codecogs.com/svg.latex?%28Q%5E%7BA%7D_%7BB%7D%29%5E%7B-1%7D" alt="(Q^{A}_{B})^{-1}" /> denotes the conjugate (or inverse) of the quaternion <img src="https://latex.codecogs.com/svg.latex?%5C%28%20Q%5E%7BA%7D_%7BB%7D%20%5C%29" alt="\( Q^{A}_{B} \)" />.

## Quaternions Interpolation slerp


## The advantages of Quaternions 

- Avoiding gimbal lock, a problem with systems such as Euler angles.
- Faster and more compact than matrices.
- Nonsingular representation (compared with Euler angles for example).



Refs: [1](https://www.youtube.com/watch?v=d4EgbgTm0Bg),[2](https://www.youtube.com/watch?v=zjMuIxRvygQ), [3](https://quaternions.online/), [4](https://www.youtube.com/watch?v=zc8b2Jo7mno), [5](https://www.youtube.com/watch?v=syQnn_xuB8U),
[6](https://www.youtube.com/watch?v=4mXL751ko0w), [7](https://www.reedbeta.com/blog/why-quaternions-double-cover/)



# Lie algebra Groups

1. <img src="https://latex.codecogs.com/svg.image?SO(3)" title="https://latex.codecogs.com/svg.image?SO(3)" />: special orthogonal group <img src="https://latex.codecogs.com/svg.image?SO(3)" title="https://latex.codecogs.com/svg.image?SO(3)" /> is the 
set of all <img src="https://latex.codecogs.com/svg.image?3\times3" title="https://latex.codecogs.com/svg.image?3\times3" /> real matrices R satisfying:
 - <img src="https://latex.codecogs.com/svg.image?R^TR=I" title="https://latex.codecogs.com/svg.image?R^TR=I" />
 - <img src="https://latex.codecogs.com/svg.image?det&space;R=1" title="https://latex.codecogs.com/svg.image?det R=1" />


2. <img src="https://latex.codecogs.com/svg.image?so(3)" title="https://latex.codecogs.com/svg.image?so(3)" />: the set of all 3x3 skew-symmetric real matrices is called <img src="https://latex.codecogs.com/svg.image?so(3)" title="https://latex.codecogs.com/svg.image?so(3)" />


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;[\mathbf&space;{x}&space;]{=}{\begin{bmatrix}\,\,0&\!-x_{3}&\,\,\,x_{2}\\\,\,\,x_{3}&0&\!-x_{1}\\\!-x_{2}&\,\,x_{1}&\,\,0\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle [\mathbf {x} ]{=}{\begin{bmatrix}\,\,0&\!-x_{3}&\,\,\,x_{2}\\\,\,\,x_{3}&0&\!-x_{1}\\\!-x_{2}&\,\,x_{1}&\,\,0\end{bmatrix}}}" />

because:

<img src="https://latex.codecogs.com/svg.image?[x]=-[x]^T" title="https://latex.codecogs.com/svg.image?[x]=-[x]^T" />

3. <img src="https://latex.codecogs.com/svg.image?SE(3)" title="https://latex.codecogs.com/svg.image?SE(3)" />: the special Euclidean groups <img src="https://latex.codecogs.com/svg.image?SE(3)" title="https://latex.codecogs.com/svg.image?SE(3)" />
is the set of all <img src="https://latex.codecogs.com/svg.image?4&space;\times&space;4" title="https://latex.codecogs.com/svg.image?4 \times 4" /> real matrices of <img src="https://latex.codecogs.com/svg.image?T&space;" title="https://latex.codecogs.com/svg.image?T " /> f the form:

- <img src="https://latex.codecogs.com/svg.image?T=\begin{bmatrix}&space;R&&space;p&space;\\&space;0&&space;1&space;\\\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?T=\begin{bmatrix} R& p \\ 0& 1 \\\end{bmatrix} " />




- <img src="https://latex.codecogs.com/svg.image?R\in&space;SO(3),&space;p\in&space;R^3" title="https://latex.codecogs.com/svg.image?R\in SO(3), p\in R^3" />


- <img src="https://latex.codecogs.com/svg.image?T^{-1=}\begin{bmatrix}&space;R^T&&space;-pR^T&space;\\&space;0&&space;1&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?T^{-1=}\begin{bmatrix} R^T& -pR^T \\ 0& 1 \\\end{bmatrix}" />

to apply a transformation, first we apply the rotation around the axis of the frame the we pre-multiplied and then we translate again on the axis of the frame that we pre-multiplied

<img src="https://latex.codecogs.com/svg.image?p=[0,&space;2&space;,0]^T\\\hat{\omega}&space;=&space;\hat{z}\\\theta=90\\T=Translate(p)Rot(\hat{\omega},\theta)&space;" title="https://latex.codecogs.com/svg.image?p=[0, 2 ,0]^T\\\hat{\omega} = \hat{z}\\\theta=90\\T=Translate(p)Rot(\hat{\omega},\theta) " />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\acute{T}=TT_{sb}" title="https://latex.codecogs.com/svg.image?\acute{T}=TT_{sb}" />

<br/>
<br/>

<img width="300" height="200" src="images/transformtion_1.jpg" />
<br/>
<br/>
<img  width="300" height="200" src="images/transformtion_2.jpg" />
<br/>
<br/>
<img  width="300" height="200" src="images/transformtion_3.jpg" />
<br/>
<br/>



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


# Conversion between different representations

Full list of conversion [here](http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm) 
