## FilterPy
Refs: [1](https://filterpy.readthedocs.io/en/latest/)



# EKF Implementations

Refs: [1](https://github.com/Sina-Baharlou/Pose-Estimation-EKF), [2](https://orocos.org/bfl.html)


# EKF for Differential Drive Robot

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-\frac{V}{\omega}&space;\sin(\phi)&space;&space;\\-\frac{V}{\omega}&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;\frac{V}{\omega}&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}}_{g(u_t,\mu_{t-1})}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi+\omega \delta t) -\frac{V}{\omega} \sin(\phi) \\-\frac{V}{\omega} \cos(\phi +\omega \delta t) +\frac{V}{\omega} \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}}_{g(u_t,\mu_{t-1})} " />



## State Space
So at the beginning we have only robot pose (robot start in its own frame) and orientation and zero land mark, as the robot explor we will add landmarks and increase the state space, but here we assume that all landmark are known beforehand, so for a map with  <img src="https://latex.codecogs.com/svg.image?n" title="https://latex.codecogs.com/svg.image?n" /> landmarks: <img src="https://latex.codecogs.com/svg.image?(3&plus;2n)" title="https://latex.codecogs.com/svg.image?(3+2n)" />  dimensional Gaussian

<img src="https://latex.codecogs.com/svg.image?x_t=\begin{pmatrix}x&space;\\y&space;\\\theta&space;\\&space;m_{1,x}&space;\\m_{1,y}&space;\\\vdots&space;\\m_{n,x}&space;\\m_{n,y}&space;\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?x_t=\begin{pmatrix}x \\y \\\theta \\ m_{1,x} \\m_{1,y} \\\vdots \\m_{n,x} \\m_{n,y} \\\end{pmatrix}" />

<br/>
<br/>


Compact Representation:

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}x&space;\\m\end{pmatrix}}_\mu&space;" title="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}x \\m\end{pmatrix}}_\mu " />


<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}\Sigma_{xx}&space;&&space;\Sigma_{xm}&space;\\\Sigma_{mx}&space;&&space;\Sigma_{mm}\end{pmatrix}}_\Sigma&space;" title="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}\Sigma_{xx} & \Sigma_{xm} \\\Sigma_{mx} & \Sigma_{mm}\end{pmatrix}}_\Sigma " />


<br/>
<br/>



Refs: [1](https://www.youtube.com/watch?v=hN8dL55rP5I), [2](https://www.mathworks.com/help/fusion/ug/pose-estimation-from-asynchronous-sensors.html#d124e16816), [3](https://www.mathworks.com/help/fusion/ug/imu-and-gps-fusion-for-inertial-navigation.html)



# STATE ESTIMATION FOR ROBOTICS

Ref: [1](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf)


