# Kinematics of Differential Drive Robots and Wheel odometry

# 1. Velocity-based (dead reckoning)
Linear Velocity:

<img src="https://latex.codecogs.com/svg.image?V=R\omega" title="https://latex.codecogs.com/svg.image?V=R\omega" />

<br/>
<br/>

so for the right wheel:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?V_r=\omega(R&plus;\frac{L}{2})" title="https://latex.codecogs.com/svg.image?V_r=\omega(R+\frac{L}{2})" />
<br/>
<br/>

and for the left wheel:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?V_l=\omega(R-\frac{L}{2})" title="https://latex.codecogs.com/svg.image?V_l=\omega(R-\frac{L}{2})" />


if we add and subtract the above equation:

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\omega=&space;\frac{V_l&space;&plus;&space;V_r}{2R}" title="https://latex.codecogs.com/svg.image?\omega= \frac{V_l + V_r}{2R}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\omega=&space;\frac{V_l&space;-&space;V_r}{L}" title="https://latex.codecogs.com/svg.image?\omega= \frac{V_l - V_r}{L}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\omega=L&space;\frac{V_l&space;&plus;&space;V_r}{2(V_r-V_l)}" title="https://latex.codecogs.com/svg.image?\omega=L \frac{V_l + V_r}{2(V_r-V_l)}" />

<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?I_{CC}=\left\{\begin{matrix}I_{CC_X}=x-R&space;\cos(\frac{\pi}{2}-\phi)&space;=x-R\sin(\phi)\\I_{CC_Y}=y-R&space;\sin(\frac{\pi}{2}-\phi)&space;=y&plus;R\cos(\phi)\end{matrix}\right." title="https://latex.codecogs.com/svg.image?I_{CC}=\left\{\begin{matrix}I_{CC_X}=x-R \cos(\frac{\pi}{2}-\phi) =x-R\sin(\phi)\\I_{CC_Y}=y-R \sin(\frac{\pi}{2}-\phi) =y+R\cos(\phi)\end{matrix}\right." />


<br/>
<br/>

<img src="images/Differential_Drive_Kinematics_of_a_Wheeled_Mobile_Robot.svg" />

<br/>
<br/>

## 1.1 Forward Kinematics for Differential Drive Robots

so we express the pose of the robot in <img src="https://latex.codecogs.com/svg.image?ICC" title="https://latex.codecogs.com/svg.image?ICC" /> frame, we rotate it around that <img src="https://latex.codecogs.com/svg.image?\omega&space;\delta&space;t" title="https://latex.codecogs.com/svg.image?\omega \delta t" /> degree and finally express it our world coordinate frame:


<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\end{bmatrix}=\begin{bmatrix}\cos(\omega&space;\delta&space;t)&space;&&space;-\sin(\omega&space;\delta&space;t)&space;&space;\\\sin(\omega&space;\delta&space;t)&space;&space;&&space;&space;\cos(\omega&space;\delta&space;t)&space;\\\end{bmatrix}\begin{bmatrix}x-I_{CC_X}\\y-I_{CC_Y}\end{bmatrix}&plus;\begin{bmatrix}I_{CC_X}&space;\\I_{CC_y}\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\end{bmatrix}=\begin{bmatrix}\cos(\omega \delta t) & -\sin(\omega \delta t) \\\sin(\omega \delta t) & \cos(\omega \delta t) \\\end{bmatrix}\begin{bmatrix}x-I_{CC_X}\\y-I_{CC_Y}\end{bmatrix}+\begin{bmatrix}I_{CC_X} \\I_{CC_y}\end{bmatrix}" />

<br/>
<br/>

If we add orientation and write all positions in our world coordinate:
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}\cos(\omega&space;\delta&space;t)&space;&&space;-\sin(\omega&space;\delta&space;t)&space;&&space;0\\\sin(\omega&space;\delta&space;t)&space;&&space;\cos(\omega&space;\delta&space;t\theta&space;&&space;0&space;\\0&space;&&space;1&space;&&space;1\end{bmatrix}\begin{bmatrix}x-x&plus;R&space;\sin&space;\phi&space;\\y-y-R&space;\cos&space;\phi&space;\\\phi\end{bmatrix}&plus;\begin{bmatrix}x-R&space;\sin&space;\phi&space;\\y&plus;R&space;\cos&space;\phi&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}\cos(\omega \delta t) & -\sin(\omega \delta t) & 0\\\sin(\omega \delta t) & \cos(\omega \delta t\theta & 0 \\0 & 1 & 1\end{bmatrix}\begin{bmatrix}x-x+R \sin \phi \\y-y-R \cos \phi \\\phi\end{bmatrix}+\begin{bmatrix}x-R \sin \phi \\y+R \cos \phi \\\omega \delta t\end{bmatrix}" />
<br/>
<br/>









<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}R&space;(\cos&space;(\omega&space;\delta&space;t)&space;&space;\sin&space;(\phi&space;)&plus;&space;\sin&space;(\omega&space;\delta&space;t&space;)\cos&space;(\phi)&space;&space;)&space;\\-R&space;(\sin(\omega&space;\delta&space;t)&space;\sin(\phi)&space;&plus;&space;cos(\omega&space;\delta&space;t)&space;cos(\phi))&space;&space;\\0&space;\end{bmatrix}&plus;\begin{bmatrix}x-R&space;\sin(\phi)&space;&space;\\y&plus;R&space;\cos(\phi)&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}R (\cos (\omega \delta t) \sin (\phi )+ \sin (\omega \delta t )\cos (\phi) ) \\-R (\sin(\omega \delta t) \sin(\phi) + cos(\omega \delta t) cos(\phi)) \\0 \end{bmatrix}+\begin{bmatrix}x-R \sin(\phi) \\y+R \cos(\phi) \\\omega \delta t\end{bmatrix}" />


<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}R\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-R&space;\sin(\phi)&space;&space;\\-R&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;R&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}R\sin(\phi+\omega \delta t) -R \sin(\phi) \\-R \cos(\phi +\omega \delta t) +R \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}" />



<br/>
<br/>
since we had :
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?R=\frac{V}{\omega}" title="https://latex.codecogs.com/svg.image?R=\frac{V}{\omega}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}\frac{V}{\omega}\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-\frac{V}{\omega}&space;\sin(\phi)&space;&space;\\-\frac{V}{\omega}&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;\frac{V}{\omega}&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}\frac{V}{\omega}\sin(\phi+\omega \delta t) -\frac{V}{\omega} \sin(\phi) \\-\frac{V}{\omega} \cos(\phi +\omega \delta t) +\frac{V}{\omega} \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}" />

<br/>
<br/>

## 1.2.  Inverse Kinematics of Differential Drive Robots



Refs: [1](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)



# 2. Odometry-based


<img src="images/odometry_model.jpg" width="461" height="192">

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\delta_{translation}=\sqrt{(\bar{x^\prime}&space;-\bar{x})^2&space;&plus;&space;(\bar{y^\prime}&space;-\bar{y})^2&space;}" title="https://latex.codecogs.com/svg.image?\delta_{translation}=\sqrt{(\bar{x^\prime} -\bar{x})^2 + (\bar{y^\prime} -\bar{y})^2 }" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\delta_{rot1}=atan2(&space;&space;\bar{y^\prime}&space;-\bar{y}&space;,&space;&space;\bar{x^\prime}&space;-\bar{x})&space;&space;)&space;&space;&space;-\bar\theta&space;" title="https://latex.codecogs.com/svg.image?\delta_{rot1}=atan2( \bar{y^\prime} -\bar{y} , \bar{x^\prime} -\bar{x}) ) -\bar\theta " />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\delta_{rot2}=\bar\theta&space;^\prime&space;&space;&space;-\bar\theta&space;-\delta_{rot1}" title="https://latex.codecogs.com/svg.image?\delta_{rot2}=\bar\theta ^\prime -\bar\theta -\delta_{rot1}" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\left\{\begin{matrix}x_t=x_{t-1}&plus;\delta_{translation}&space;&space;&space;\cos(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;)&space;\\y_t=y_{t-1}&plus;\delta_{translation}&space;&space;&space;\sin(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;)&space;\\\theta_t=\theta_{t-1}&plus;\delta_{rot1}&plus;\delta_{rot2}\end{matrix}\right.&space;" title="https://latex.codecogs.com/svg.image?\left\{\begin{matrix}x_t=x_{t-1}+\delta_{translation} \cos( \theta_{t-1} +\delta{rot1} ) \\y_t=y_{t-1}+\delta_{translation} \sin( \theta_{t-1} +\delta{rot1} ) \\\theta_t=\theta_{t-1}+\delta_{rot1}+\delta_{rot2}\end{matrix}\right. " />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?g(u_t,\mu_{t-1})=\begin{pmatrix}x_t&space;\\y_t&space;\\\theta_t\end{pmatrix}&space;=\begin{pmatrix}x_{t-1}\\y_{t-1}&space;\\\theta_{t-1}\end{pmatrix}&space;&plus;\begin{pmatrix}&space;\delta_{translation}&space;\cos(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;&space;&space;\\&space;\delta_{translation}&space;\sin(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;&space;\\&space;\delta_{rot1}&plus;\delta_{rot2}\end{pmatrix}&space;" title="https://latex.codecogs.com/svg.image?g(u_t,\mu_{t-1})=\begin{pmatrix}x_t \\y_t \\\theta_t\end{pmatrix} =\begin{pmatrix}x_{t-1}\\y_{t-1} \\\theta_{t-1}\end{pmatrix} +\begin{pmatrix} \delta_{translation} \cos( \theta_{t-1} +\delta{rot1} \\ \delta_{translation} \sin( \theta_{t-1} +\delta{rot1} \\ \delta_{rot1}+\delta_{rot2}\end{pmatrix} " />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?G_t=&space;\frac{\partial}{\partial(x,y,\theta)^T}&space;\begin{bmatrix}&space;\begin{pmatrix}x\\y&space;\\\theta\end{pmatrix}&space;&plus;\begin{pmatrix}&space;\delta_{translation}&space;\cos(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{translation}&space;\sin(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{rot1}&plus;\delta_{rot2}\end{pmatrix}&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?G_t= \frac{\partial}{\partial(x,y,\theta)^T} \begin{bmatrix} \begin{pmatrix}x\\y \\\theta\end{pmatrix} +\begin{pmatrix} \delta_{translation} \cos( \theta +\delta{rot1} )\\ \delta_{translation} \sin( \theta +\delta{rot1} )\\ \delta_{rot1}+\delta_{rot2}\end{pmatrix} \end{bmatrix}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?G_t=I&plus;&space;\frac{\partial}{\partial(x,y,\theta)^T}&space;\begin{bmatrix}&space;\begin{pmatrix}&space;\delta_{translation}&space;\cos(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{translation}&space;\sin(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{rot1}&plus;\delta_{rot2}\end{pmatrix}&space;\end{pmatrix}" title="https://latex.codecogs.com/svg.image?G_t=I+ \frac{\partial}{\partial(x,y,\theta)^T} \begin{bmatrix} \begin{pmatrix} \delta_{translation} \cos( \theta +\delta{rot1} )\\ \delta_{translation} \sin( \theta +\delta{rot1} )\\ \delta_{rot1}+\delta_{rot2}\end{pmatrix} \end{pmatrix}" />



<br/>
<br/>
Not sure about the followings:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?G_t=I&plus;&space;\begin{pmatrix}0&space;&&space;0&space;&&space;-\delta_{translation}&space;\sin(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;0&space;&&space;\delta_{translation}&space;\cos(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;0&space;&&space;0&space;\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?G_t=I+ \begin{pmatrix}0 & 0 & -\delta_{translation} \sin(\theta+\delta rot1) \\0 & 0 & \delta_{translation} \cos(\theta+\delta rot1) \\0 & 0 & 0 \\\end{pmatrix}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?G_t=\begin{pmatrix}1&space;&&space;0&space;&&space;-\delta_{translation}&space;\sin(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;1&space;&&space;\delta_{translation}&space;\cos(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;0&space;&&space;1&space;\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?G_t=\begin{pmatrix}1 & 0 & -\delta_{translation} \sin(\theta+\delta rot1) \\0 & 1 & \delta_{translation} \cos(\theta+\delta rot1) \\0 & 0 & 1 \\\end{pmatrix}" />
<br/>
<br/>



<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?h(\bar{\mu}_t,j)=z_t^i=\begin{pmatrix}r_i^t&space;\\\phi_i^t\end{pmatrix}=\begin{pmatrix}\sqrt{(&space;\bar{\mu}_{j,x}&space;&space;-\bar{\mu}_{t,x}&space;)^2&space;&plus;&space;(&space;\bar{\mu}_{j,y}&space;&space;-\bar{\mu}_{t,y}&space;&space;&space;)^2&space;}&space;&space;&space;\\atan2(&space;\bar{\mu}_{j,y}&space;-\bar{\mu}_{t,y}&space;&space;&space;&space;,&space;&space;&space;\bar{\mu}_{j,x}-&space;\bar{\mu}_{t,x}&space;&space;)-&space;\bar{\mu}_{t,\theta}&space;)\end{pmatrix}" title="https://latex.codecogs.com/svg.image?h(\bar{\mu}_t,j)=z_t^i=\begin{pmatrix}r_i^t \\\phi_i^t\end{pmatrix}=\begin{pmatrix}\sqrt{( \bar{\mu}_{j,x} -\bar{\mu}_{t,x} )^2 + ( \bar{\mu}_{j,y} -\bar{\mu}_{t,y} )^2 } \\atan2( \bar{\mu}_{j,y} -\bar{\mu}_{t,y} , \bar{\mu}_{j,x}- \bar{\mu}_{t,x} )- \bar{\mu}_{t,\theta} )\end{pmatrix}" />


<br/>
<br/>

The pose of the <img src="https://latex.codecogs.com/svg.image?j^{th}" title="https://latex.codecogs.com/svg.image?j^{th}" /> landmark

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}\bar{\mu}_{j,x}&space;\\\bar{\mu}_{j,y}\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}\bar{\mu}_{j,x} \\\bar{\mu}_{j,y}\end{bmatrix}" />

<br/>
<br/>

the pose of the robot at time t:
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;\bar{\mu}_{t,x}&space;\\&space;\bar{\mu}_{t,y}&space;\\&space;\bar{\mu}_{t,\theta}\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} \bar{\mu}_{t,x} \\ \bar{\mu}_{t,y} \\ \bar{\mu}_{t,\theta}\end{bmatrix} " />
<br/>
<br/>


observed range and bearing of the landmark:


<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}r_t^i&space;\\\phi_t^i\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}r_t^i \\\phi_t^i\end{bmatrix} " />



# DiffBot Differential Drive Mobile Robot

[ros-mobile-robots](https://ros-mobile-robots.com/)


