# 1. Global References

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



# 2. Accelerometer Model


<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%20a_x%20%5C%5C%20a_y%5C%5C%20a_z%20%5Cend%7Bbmatrix%7D%3D%20%5Cunderbrace%7B%5Cfrac%7Bdv%7D%7Bdt%7D%7D_%7B%5Ctext%7Blinear%7D%7D%20&plus;%20%5Cunderbrace%7B%5Comega_b%20%5Ctimes%20%5Cnu%7D_%7B%5Ctext%7Btranslation%2C%20rotation%7D%7D%20-%5Cunderbrace%7BR%5Cbegin%7Bbmatrix%7D0%5C%5C%200%5C%5C%20g%20%5Cend%7Bbmatrix%7D%7D_%7B%5Ctext%7Bgravity%7D%7D%20&plus;%20%5Cunderbrace%7B%5Cbeta%28t%29%7D_%7B%5Ctext%7Bbias%7D%7D%20&plus;%20%5Cunderbrace%7B%20%5Cmu%28t%29%7D_%7B%5Ctext%7Bnoise%7D%7D" alt="https://latex.codecogs.com/svg.latex?\begin{bmatrix} a_x \\  a_y\\  a_z \end{bmatrix}= \underbrace{\frac{dv}{dt}}_{\text{linear}} +  \underbrace{\omega_b \times \nu}_{\text{translation, rotation}}  -\underbrace{R\begin{bmatrix}0\\ 0\\  g \end{bmatrix}}_{\text{gravity}}  + \underbrace{\beta(t)}_{\text{bias}} + \underbrace{ \mu(t)}_{\text{noise}}" />

# 3. Gyroscope Model

<img src="https://latex.codecogs.com/svg.latex?%5Comega%3D%5Comega_%7Btrue%7D%20&plus;%20%5Cunderbrace%7B%5Cbeta%28t%29%7D_%7B%5Ctext%7Bbias%7D%7D%20&plus;%20%5Cunderbrace%7B%20%5Cmu%28t%29%7D_%7B%5Ctext%7Bnoise%7D%7D" alt="https://latex.codecogs.com/svg.latex?\omega=\omega_{true}  + \underbrace{\beta(t)}_{\text{bias}} + \underbrace{ \mu(t)}_{\text{noise}} " />





# 4. Attitude from gravity (Tilt)



Accelerometer sensors measure the difference between any linear acceleration in the accelerometer’s reference frame and can be used to determine the accelerometer pitch and roll orientation angles.
the accelerometer will give us <img src="https://latex.codecogs.com/svg.latex?G_p" alt="https://latex.codecogs.com/svg.latex?G_p" />
and we are looking for <img src="https://latex.codecogs.com/svg.latex?R" alt="https://latex.codecogs.com/svg.latex?R" /> 
that relate gravity to out measurement to extract roll and pitch.


**Since the gravity express as <img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\begin{bmatrix} 0\\  0\\  1 \end{bmatrix}" /> it means that the `Z` axis of  world coordinate frame is up, East, North, Up (ENU), used in geography (z is up and x is in the direction of move, y is pointing left)**


```
          ▲ z
          |    ▲
          |   / x
          |  /
y         | /
◀---------|/
```


<img src="https://latex.codecogs.com/svg.latex?G_p%3D%5Cbegin%7Bbmatrix%7D%20G_%7Bpx%7D%20%5C%5C%20G_%7Bpy%7D%20%5C%5C%20G_%7Bpz%7D%20%5Cend%7Bbmatrix%7D%3DRg%3DR%5Cbegin%7Bbmatrix%7D%200%20%5C%5C%200%20%5C%5C%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?G_p=\begin{bmatrix} G_{px} \\ G_{py} \\  G_{pz} \end{bmatrix}=Rg=R\begin{bmatrix} 0 \\ 0 \\  1 \end{bmatrix}" />





<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20R_x%28%5Cphi%29%3D%5Cbegin%7Bpmatrix%7D%201%20%26%200%20%260%20%5C%5C%200%20%26cos%28%5Cphi%29%20%26%20sin%28%5Cphi%29%20%5C%5C%200%20%26%20-sin%28%5Cphi%29%20%26%20cos%28%5Cphi%29%5C%5C%20%5Cend%7Bpmatrix%7D%20%5C%5C%20R_y%28%5Ctheta%29%3D%5Cbegin%7Bpmatrix%7D%20cos%28%5Ctheta%29%20%260%20%26-sin%28%5Ctheta%29%5C%5C%200%26%201%26%200%5C%5C%20sin%28%5Ctheta%29%20%260%20%26cos%28%5Ctheta%29%20%5C%5C%20%5Cend%7Bpmatrix%7D%20%5C%5C%20R_z%28%5Cpsi%29%3D%5Cbegin%7Bpmatrix%7D%20cos%28%5Cpsi%29%20%26%20sin%28%5Cpsi%29%20%260%20%5C%5C%20-sin%28%5Cpsi%29%20%26%20cos%28%5Cpsi%29%20%260%20%5C%5C%200%20%26%200%20%26%201%5C%5C%20%5Cend%7Bpmatrix%7D" alt="\\R_x(\phi)=\begin{pmatrix} 
1 & 0 &0 \\  0 &cos(\phi)  & sin(\phi) \\ 0 & -sin(\phi) & cos(\phi)\\ \end{pmatrix}\\ R_y(\theta)=\begin{pmatrix}  cos(\theta) &0 &-sin(\theta)\\  0& 1& 0\\ sin(\theta) &0 &cos(\theta) \\ \end{pmatrix} \\ R_z(\psi)=\begin{pmatrix} cos(\psi) & sin(\psi) &0 \\  -sin(\psi) & cos(\psi) &0 \\ 0 & 0 & 1\\ \end{pmatrix}" />




We have six different rotation matrix depending on the order of rotation around axis:

<img src="https://latex.codecogs.com/svg.latex?R_%7Bxyz%7D%2CR_%7Bxzy%7D%2CR_%7Byxz%7D%2CR_%7Byzx%7D%2CR_%7Bzyx%7D%2CR_%7Bzxy%7D%2C" alt="https://latex.codecogs.com/svg.latex?R_{xyz},R_{xzy},R_{yxz},R_{yzx},R_{zyx},R_{zxy}" /> and in 4 of them we will have to determine 
<img src="https://latex.codecogs.com/svg.image?\phi,&space;\theta,&space;\psi" title="https://latex.codecogs.com/svg.image?\phi, \theta, \psi" />, 

<br/>

For instance in 
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?R_zR_yR_x%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%20%5Cend%7Bbmatrix%7D%20%3D%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%5C%5C-%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20&plus;%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?R_zR_yR_x\begin{bmatrix} 0\\ 0\\  1 \end{bmatrix} =\displaystyle \left[\begin{matrix}\cos{\left(\psi \right)} \cos{\left(\theta \right)} & \sin{\left(\phi \right)} \sin{\left(\theta \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \cos{\left(\phi \right)} & \sin{\left(\phi \right)} \sin{\left(\psi \right)} - \sin{\left(\theta \right)} \cos{\left(\phi \right)} \cos{\left(\psi \right)}\\- \sin{\left(\psi \right)} \cos{\left(\theta \right)} & - \sin{\left(\phi \right)} \sin{\left(\psi \right)} \sin{\left(\theta \right)} + \cos{\left(\phi \right)} \cos{\left(\psi \right)} & \sin{\left(\phi \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \sin{\left(\theta \right)} \cos{\left(\phi \right)}\\\sin{\left(\theta \right)} & - \sin{\left(\phi \right)} \cos{\left(\theta \right)} & \cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]\begin{bmatrix} 0\\ 0\\  1 \end{bmatrix} "/>


<br/>
<br/>



<img src="https://latex.codecogs.com/svg.latex?%3D%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%5C%5C%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D" alt="https://latex.codecogs.com/svg.latex?=\displaystyle \left[\begin{matrix}\sin{\left(\phi \right)} \sin{\left(\psi \right)} - \sin{\left(\theta \right)} \cos{\left(\phi \right)} \cos{\left(\psi \right)}\\\sin{\left(\phi \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \sin{\left(\theta \right)} \cos{\left(\phi \right)}\\\cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]" />

<br/>
<br/>
However in the following rotation matrix, we have to determine only <img src="https://latex.codecogs.com/svg.image?\phi,&space;\theta" title="https://latex.codecogs.com/svg.image?\phi, \theta" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?R_%7Bxyz%7D%5Cbegin%7Bbmatrix%7D0%5C%5C%200%5C%5C%201%5Cend%7Bbmatrix%7D%3D%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20-%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20&plus;%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%26%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?R_{xyz}\begin{bmatrix}0\\ 0\\ 1\end{bmatrix}=\displaystyle \left[\begin{matrix}\cos{\left(\psi \right)} \cos{\left(\theta \right)} & \sin{\left(\psi \right)} \cos{\left(\theta \right)} & - \sin{\left(\theta \right)}\\\sin{\left(\phi \right)} \sin{\left(\theta \right)} \cos{\left(\psi \right)} - \sin{\left(\psi \right)} \cos{\left(\phi \right)} & \sin{\left(\phi \right)} \sin{\left(\psi \right)} \sin{\left(\theta \right)} + \cos{\left(\phi \right)} \cos{\left(\psi \right)} & \sin{\left(\phi \right)} \cos{\left(\theta \right)}\\\sin{\left(\phi \right)} \sin{\left(\psi \right)} + \sin{\left(\theta \right)} \cos{\left(\phi \right)} \cos{\left(\psi \right)} & - \sin{\left(\phi \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \sin{\left(\theta \right)} \cos{\left(\phi \right)} & \cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]\begin{bmatrix} 0\\  0\\  1\end{bmatrix}" />



<img src="https://latex.codecogs.com/svg.latex?=%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D" alt="https://latex.codecogs.com/svg.latex?=\displaystyle \left[\begin{matrix}- \sin{\left(\theta \right)}\\\sin{\left(\phi \right)} \cos{\left(\theta \right)}\\\cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]" />



<br/>
<br/>

Therefore if we write R as <img src="https://latex.codecogs.com/svg.latex?R_%7Bxyz%7D%20%5Ctext%7B%20or%20%7D%20R_%7Byxz%7D" alt="https://latex.codecogs.com/svg.latex?R_{xyz} \text{ or }  R_{yxz}"/>:


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;G_x\\&space;G_y&space;\\&space;G_z\end{bmatrix}=R_{xyz}\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=R_x(\phi)R_y(\theta)R_z(\psi)\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=\begin{bmatrix}&space;-sin\theta\\&space;cos\theta&space;sin\phi\\cos\theta&space;cos\phi\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} G_x\\ G_y \\ G_z\end{bmatrix}=R_{xyz}\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=R_x(\phi)R_y(\theta)R_z(\psi)\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=\begin{bmatrix} -sin\theta\\ cos\theta sin\phi\\cos\theta cos\phi\end{bmatrix}" />

or 

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;G_x\\&space;G_y&space;\\&space;G_z\end{bmatrix}=R_{yxz}\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=R_y(\theta)R_x(\phi)R_z(\psi)\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=\begin{bmatrix}&space;-sin\theta&space;cos\phi&space;\\sin\phi&space;\\cos\theta&space;cos\phi\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} G_x\\ G_y \\ G_z\end{bmatrix}=R_{yxz}\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=R_y(\theta)R_x(\phi)R_z(\psi)\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=\begin{bmatrix} -sin\theta cos\phi \\sin\phi \\cos\theta cos\phi\end{bmatrix}" />


The lack of any dependence on the yaw rotation angle <img src="https://latex.codecogs.com/svg.image?&space;\psi" title="https://latex.codecogs.com/svg.image? \psi" /> is easy to understand physically
since the first rotation is in yaw <img src="https://latex.codecogs.com/svg.image?&space;\psi" title="https://latex.codecogs.com/svg.image? \psi" /> around the smartphone z-axis which is initially aligned with the
gravitational field and pointing downwards. All accelerometers are completely insensitive to rotations
about the gravitational field vector and cannot be used to determine such a rotation.

It is conventional therefore to select either the rotation sequence <img src="https://latex.codecogs.com/svg.image?R_{xyz}" title="https://latex.codecogs.com/svg.image?R_{xyz}" /> or <img src="https://latex.codecogs.com/svg.image?R_{yxz}" title="https://latex.codecogs.com/svg.image?R_{yxz}" />

<br/>
<br/>


Refs: [1](https://ahrs.readthedocs.io/en/latest/filters/tilt.html)


## 4.1. Solving R xyz for the Pitch and Roll Angles

<img src="https://latex.codecogs.com/svg.image?R_{xyz}" title="https://latex.codecogs.com/svg.image?R_{xyz}" /> will give us:

<img src="https://latex.codecogs.com/svg.image?\frac{G}{||G&space;||}&space;=\begin{bmatrix}&space;-sin\theta\\&space;cos\theta&space;sin\phi\\cos\theta&space;cos\phi\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\frac{G}{||G ||} =\begin{bmatrix} -sin\theta\\ cos\theta sin\phi\\cos\theta cos\phi\end{bmatrix}" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?tan\phi=\frac{G_y}{G_z}" title="https://latex.codecogs.com/svg.image?tan\phi=\frac{G_y}{G_z}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{&space;\sqrt{&space;G_z^2&space;&plus;&space;G_y^2}}" title="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{ \sqrt{ G_z^2 + G_y^2}}" />
<br/>
<br/>


## 4.2 Solving R yxz for the Pitch and Roll Angles

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?R_{yxz}" title="https://latex.codecogs.com/svg.image?R_{yxz}" /> will give us:

<img src="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{G_z}" title="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{G_z}" />
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?tan\theta=\frac{G_y}{&space;\sqrt{&space;G_z^2&space;&plus;&space;G_x^2}}" title="https://latex.codecogs.com/svg.image?tan\theta=\frac{G_y}{ \sqrt{ G_z^2 + G_x^2}}" />

<br/>
<br/>

**The order of rotations is important and must always be specified when referring to specific orientation angles.**

Refs: [1](https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf)



# Expressing IMU reading with Quaternion

The IMU (local sensor) frame is labeled as <img src="https://latex.codecogs.com/svg.latex?L" alt="https://latex.codecogs.com/svg.latex?L" /> , and the global (Earth) frame as <img src="https://latex.codecogs.com/svg.latex?G" alt="https://latex.codecogs.com/svg.latex?G" /> . The measured acceleration, <img src="https://latex.codecogs.com/svg.latex?%5EL%5Cmathbf%7Ba%7D" alt="https://latex.codecogs.com/svg.latex?^L\mathbf{a}" /> , and the true Earth gravitational acceleration, <img src="https://latex.codecogs.com/svg.latex?%5EG%5Cmathbf%7Bg%7D" alt="https://latex.codecogs.com/svg.latex?^G\mathbf{g}" /> , are defined as unit vectors:





<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Barray%7D%7Brcl%7D%20%5EL%5Cmathbf%7Ba%7D%20%26%3D%26%20%5Cbegin%7Bbmatrix%7Da_x%20%26%20a_y%20%26%20a_z%5Cend%7Bbmatrix%7D%5ET%20%5C%5C%20%26%26%20%5C%5C%20%5EG%5Cmathbf%7Bg%7D%20%26%3D%26%20%5Cbegin%7Bbmatrix%7D0%20%26%200%20%26%201%5Cend%7Bbmatrix%7D%5ET%20%5Cend%7Barray%7D" alt="https://latex.codecogs.com/svg.latex?\begin{array}{rcl} ^L\mathbf{a} &=& \begin{bmatrix}a_x & a_y & a_z\end{bmatrix}^T \\ && \\ ^G\mathbf{g} &=& \begin{bmatrix}0 & 0 & 1\end{bmatrix}^T \end{array}" />

The gyroscopes measure the angular velocity (are not normalized unlike the other sensors), and are usually expressed in radians per second.

<img src="https://latex.codecogs.com/svg.latex?%5EL%5Cmathbf%7B%5Comega%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%5Comega_x%20%26%20%5Comega_y%20%26%20%5Comega_z%5Cend%7Bbmatrix%7D%5ET" alt="https://latex.codecogs.com/svg.latex?^L\mathbf{\omega} = \begin{bmatrix}\omega_x & \omega_y & \omega_z\end{bmatrix}^T" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?%5EL_G%5Cmathbf%7Bq%7D" alt="https://latex.codecogs.com/svg.latex?^L_G\mathbf{q}" />:
relats the global frame <img src="https://latex.codecogs.com/svg.latex?G" alt="https://latex.codecogs.com/svg.latex?G" /> to the local frame <img src="https://latex.codecogs.com/svg.latex?L" alt="https://latex.codecogs.com/svg.latex?L" /> through the inverse orientation which rotates the measured quantities <img src="https://latex.codecogs.com/svg.latex?%5EL%5Cmathbf%7Ba%7D" alt="https://latex.codecogs.com/svg.latex?^L\mathbf{a}" />
  into the reference quantities <img src="https://latex.codecogs.com/svg.latex?%5EG%5Cmathbf%7Bg%7D" alt="https://latex.codecogs.com/svg.latex?^G\mathbf{g}" />:



<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Barray%7D%7Brcl%7D%20%5Cmathbf%7BR%7D%5ET%28%5EL_G%5Cmathbf%7Bq%7D%29%5C%2C%5EL%5Cmathbf%7Ba%7D%20%26%3D%26%20%5C%2C%5EG%5Cmathbf%7Bg%7D%20%5Cend%7Barray%7D" alt="https://latex.codecogs.com/svg.latex?\begin{array}{rcl} \mathbf{R}^T(^L_G\mathbf{q})\,^L\mathbf{a} &=& \,^G\mathbf{g} \end{array} " /> 

<br/>

# 5. Quaternion from Accelerometer

<br/>


<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Barray%7D%7Brcl%7D%20%5Cmathbf%7BR%7D%28%5EL_G%5Cmathbf%7Bq%7D%29%5C%2C%5EG%5Cmathbf%7Bg%7D%20%26%3D%26%20%5C%2C%20%5EL%5Cmathbf%7Ba%7D%20%5C%5C%20%5Cmathbf%7BR%7D%28%5Cmathbf%7Bq%7D_%5Cmathrm%7Bacc%7D%29%20%5Cbegin%7Bbmatrix%7D0%20%5C%5C%200%20%5C%5C%201%5Cend%7Bbmatrix%7D%20%26%3D%26%20%5Cbegin%7Bbmatrix%7Da_x%20%5C%5C%20a_y%20%5C%5C%20a_z%5Cend%7Bbmatrix%7D%20%5Cend%7Barray%7D" alt="https://latex.codecogs.com/svg.latex?\begin{array}{rcl}
\mathbf{R}(^L_G\mathbf{q})\,^G\mathbf{g} &=& \, ^L\mathbf{a} \\
\mathbf{R}(\mathbf{q}_\mathrm{acc})
\begin{bmatrix}0 \\ 0 \\ 1\end{bmatrix} &=&
\begin{bmatrix}a_x \\ a_y \\ a_z\end{bmatrix}
\end{array}" />


The alignment of the gravity vector from global frame into local frame can be achieved by infinite rotations with definite roll and pitch angles and arbitrary yaw. To restrict the solutions to a finite number <img src="https://latex.codecogs.com/svg.latex?q_%7Bz%5Cmathrm%7Bacc%7D%7D%3D0" alt="https://latex.codecogs.com/svg.latex?q_{z\mathrm{acc}}=0" /> 
 is chosen.


xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

# 6. Quaternion Integration
## 6.1. Numerical Solution


<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bq%7D_%7Bt&plus;1%7D%20%3D%20%5CBigg%5B%5Cmathbf%7BI%7D_4%20&plus;%20%5Cfrac%7B1%7D%7B2%7D%5Cboldsymbol%5COmega%28%5Cboldsymbol%5Comega%29%5CDelta%20t%5CBigg%5D%5Cmathbf%7Bq%7D_t" alt="https://latex.codecogs.com/svg.latex?\mathbf{q}_{t+1} = \Bigg[\mathbf{I}_4 + \frac{1}{2}\boldsymbol\Omega(\boldsymbol\omega)\Delta t\Bigg]\mathbf{q}_t
"  />


## 6.2. Closed-form Solution

<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bq%7D_%7Bt&plus;1%7D%20%3D%20%5CBigg%5B%20%5Ccos%5CBig%28%5Cfrac%7B%5C%7C%5Cboldsymbol%5Comega%5C%7C%5CDelta%20t%7D%7B2%7D%5CBig%29%5Cmathbf%7BI%7D_4%20&plus;%20%5Cfrac%7B1%7D%7B%5C%7C%5Cboldsymbol%5Comega%5C%7C%7D%5Csin%5CBig%28%5Cfrac%7B%5C%7C%5Cboldsymbol%5Comega%5C%7C%5CDelta%20t%7D%7B2%7D%5CBig%29%5Cboldsymbol%5COmega%28%5Cboldsymbol%5Comega%29%20%5CBigg%5D%5Cmathbf%7Bq%7D_t" alt="https://latex.codecogs.com/svg.latex?\mathbf{q}_{t+1} =
\Bigg[
\cos\Big(\frac{\|\boldsymbol\omega\|\Delta t}{2}\Big)\mathbf{I}_4 +
\frac{1}{\|\boldsymbol\omega\|}\sin\Big(\frac{\|\boldsymbol\omega\|\Delta t}{2}\Big)\boldsymbol\Omega(\boldsymbol\omega)
\Bigg]\mathbf{q}_t
" />


# 7.1 Quaternion Derivative

An orientation is described by <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bq%7D%20%28t&plus;%5CDelta%20t%29" alt="https://latex.codecogs.com/svg.latex?\mathbf{q} (t+\Delta t)" />  at a time <img src="https://latex.codecogs.com/svg.latex?t&plus;%5CDelta%20t" alt="https://latex.codecogs.com/svg.latex?t+\Delta t" />t . This is after a rotation change <img src="https://latex.codecogs.com/svg.latex?%5CDelta%20q" alt="https://latex.codecogs.com/svg.latex?\Delta q" />  during <img src="https://latex.codecogs.com/svg.latex?%5CDelta%20t" alt="https://latex.codecogs.com/svg.latex?\Delta t" />  seconds is performed on the local frame 



<img src="https://latex.codecogs.com/svg.latex?%5Cdot%7B%5Cmathbf%7Bq%7D%7D%20%3D%20%5Cfrac%7B1%7D%7B2%7D%5Cboldsymbol%5COmega%28%5Cboldsymbol%5Comega%29%5Cmathbf%7Bq%7D" alt="https://latex.codecogs.com/svg.latex?\dot{\mathbf{q}} = \frac{1}{2}\boldsymbol\Omega(\boldsymbol\omega)\mathbf{q}" />
<img src="" alt="" />

<img src="" alt="" />


# Relationship Between Euler-Angle Rates and Body-Axis Rates

<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%20%5Cdot%7B%5Cphi%7D%20%5C%5C%20%5Cdot%7B%5Ctheta%7D%5C%5C%20%5Cdot%7B%5Cpsi%7D%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%201%20%26%20sin%28%5Cphi%29tan%28%5Ctheta%29%20%26%20cos%28%5Cphi%29%5Ctan%28%5Ctheta%29%20%5C%5C%200%20%26%20cos%28%5Cphi%29%20%26%20-sin%28%5Cphi%29%5C%5C%200%20%26%20sin%28%5Cphi%29sec%28%5Ctheta%29%20%26%20cos%28%5Cphi%29sec%28%5Ctheta%29%20%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7D%20p%5C%5C%20q%5C%5C%20r%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\begin{bmatrix}
\dot{\phi} \\ 
\dot{\theta}\\ 
\dot{\psi}
\end{bmatrix}=\begin{bmatrix}
1 & sin(\phi)tan(\theta) & cos(\phi)\tan(\theta) \\ 
0 & cos(\phi) & -sin(\phi)\\ 
0 & sin(\phi)sec(\theta) & cos(\phi)sec(\theta) 
\end{bmatrix}\begin{bmatrix}
p\\ 
q\\ 
r
\end{bmatrix}" />
<br/>
<br/>
# Complementary Filter

Accelerometer gives estimate in non accelerating conditions (they have problem when they are in acceleration motion) and Gyroscope gives estimate of short period of times (bias drift)


<img src="https://latex.codecogs.com/svg.latex?%5Cunderbrace%7B%5Chat%7B%5Cphi%7D_%7Bn&plus;1%7D%20%7D_%7B%5Ctext%7Bcurrent%20angle%20estimate%7D%7D%20%3D%5Cunderbrace%7B%5Chat%7B%5Cphi%7D_%7Bacc%2Cn%7D%20%7D_%7B%5Ctext%7Bcalculated%20from%20accelerometer%7D%7D.%5Calpha%20&plus;%281-%5Calpha%29%5B%5Cunderbrace%7B%5Chat%7B%5Cphi%7D_%7Bn%7D%20%7D_%7B%5Ctext%7Bprevious%20angle%20estimate%7D%7D%20&plus;%20%5Cunderbrace%7BT.%5Cdot%7B%5Cphi%7D_%7Bgyro%2Cn%7D%20%7D_%7B%5Ctext%7Bchanges%20in%20angle%7D%7D%20%5D" alt="https://latex.codecogs.com/svg.latex?\underbrace{\hat{\phi}_{n+1}  }_{\text{current angle estimate}} =\underbrace{\hat{\phi}_{acc,n}  }_{\text{calculated from accelerometer}}.\alpha +(1-\alpha)[\underbrace{\hat{\phi}_{n}  }_{\text{previous angle estimate}} + \underbrace{T.\dot{\phi}_{gyro,n}  }_{\text{changes in angle}} ]"/>




<img src="https://latex.codecogs.com/svg.latex?%5Cunderbrace%7B%5Chat%7B%5Ctheta%7D_%7Bn&plus;1%7D%20%7D_%7B%5Ctext%7Bcurrent%20angle%20estimate%7D%7D%20%3D%5Cunderbrace%7B%5Chat%7B%5Ctheta%7D_%7Bacc%2Cn%7D%20%7D_%7B%5Ctext%7Bcalculated%20from%20accelerometer%7D%7D.%5Calpha%20&plus;%281-%5Calpha%29%5B%5Cunderbrace%7B%5Chat%7B%5Ctheta%7D_%7Bn%7D%20%7D_%7B%5Ctext%7Bprevious%20angle%20estimate%7D%7D%20&plus;%20%5Cunderbrace%7BT.%5Cdot%7B%5Ctheta%7D_%7Bgyro%2Cn%7D%20%7D_%7B%5Ctext%7Bchanges%20in%20angle%7D%7D%20%5D" alt="https://latex.codecogs.com/svg.latex?\underbrace{\hat{\theta}_{n+1}  }_{\text{current angle estimate}} =\underbrace{\hat{\theta}_{acc,n}  }_{\text{calculated from accelerometer}}.\alpha +(1-\alpha)[\underbrace{\hat{\theta}_{n}  }_{\text{previous angle estimate}} + \underbrace{T.\dot{\theta}_{gyro,n}  }_{\text{changes in angle}} ]" />


<br/>
<br/>




# Quaternion-Based Complementary Filter



# Accelerometer-Based Correction






# Attitude from angular rate (Attitude propagation)
A quaternion is updated via **integration** of angular rate measurements of a gyroscope. 

matrix exponential:
<br/>

<img src="https://latex.codecogs.com/svg.latex?e%5E%5Cmathbf%7BX%7D%20%3D%20%5Csum_%7Bk%3D0%7D%5E%5Cinfty%20%5Cfrac%7B1%7D%7Bk%21%7D%20%5Cmathbf%7BX%7D%5Ek" alt="https://latex.codecogs.com/svg.latex?e^\mathbf{X} = \sum_{k=0}^\infty \frac{1}{k!} \mathbf{X}^k" />


Euler's formula:


<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20e%5E%7Bix%7D%3D%5Ccos%20x&plus;i%5Csin%20x%2C%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle e^{ix}=\cos x+i\sin x,}" />

proof of Euler's formula using Taylor series:

<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20f%28a%29&plus;%7B%5Cfrac%20%7Bf%27%28a%29%7D%7B1%21%7D%7D%28x-a%29&plus;%7B%5Cfrac%20%7Bf%27%27%28a%29%7D%7B2%21%7D%7D%28x-a%29%5E%7B2%7D&plus;%7B%5Cfrac%20%7Bf%27%27%27%28a%29%7D%7B3%21%7D%7D%28x-a%29%5E%7B3%7D&plus;%5Ccdots%20%2C%7D" alt="{\displaystyle f(a)+{\frac {f'(a)}{1!}}(x-a)+{\frac {f''(a)}{2!}}(x-a)^{2}+{\frac {f'''(a)}{3!}}(x-a)^{3}+\cdots ,}" />

if we write it for around point zero (a=0):

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%7B%5Cbegin%7Baligned%7De%5E%7Bix%7D%26%3D1&plus;ix&plus;%7B%5Cfrac%20%7B%28ix%29%5E%7B2%7D%7D%7B2%21%7D%7D&plus;%7B%5Cfrac%20%7B%28ix%29%5E%7B3%7D%7D%7B3%21%7D%7D&plus;%7B%5Cfrac%20%7B%28ix%29%5E%7B4%7D%7D%7B4%21%7D%7D&plus;%7B%5Cfrac%20%7B%28ix%29%5E%7B5%7D%7D%7B5%21%7D%7D&plus;%7B%5Cfrac%20%7B%28ix%29%5E%7B6%7D%7D%7B6%21%7D%7D&plus;%7B%5Cfrac%20%7B%28ix%29%5E%7B7%7D%7D%7B7%21%7D%7D&plus;%7B%5Cfrac%20%7B%28ix%29%5E%7B8%7D%7D%7B8%21%7D%7D&plus;%5Ccdots%20%5C%5C%5B8pt%5D%26%3D1&plus;ix-%7B%5Cfrac%20%7Bx%5E%7B2%7D%7D%7B2%21%7D%7D-%7B%5Cfrac%20%7Bix%5E%7B3%7D%7D%7B3%21%7D%7D&plus;%7B%5Cfrac%20%7Bx%5E%7B4%7D%7D%7B4%21%7D%7D&plus;%7B%5Cfrac%20%7Bix%5E%7B5%7D%7D%7B5%21%7D%7D-%7B%5Cfrac%20%7Bx%5E%7B6%7D%7D%7B6%21%7D%7D-%7B%5Cfrac%20%7Bix%5E%7B7%7D%7D%7B7%21%7D%7D&plus;%7B%5Cfrac%20%7Bx%5E%7B8%7D%7D%7B8%21%7D%7D&plus;%5Ccdots%20%5C%5C%5B8pt%5D%26%3D%5Cleft%281-%7B%5Cfrac%20%7Bx%5E%7B2%7D%7D%7B2%21%7D%7D&plus;%7B%5Cfrac%20%7Bx%5E%7B4%7D%7D%7B4%21%7D%7D-%7B%5Cfrac%20%7Bx%5E%7B6%7D%7D%7B6%21%7D%7D&plus;%7B%5Cfrac%20%7Bx%5E%7B8%7D%7D%7B8%21%7D%7D-%5Ccdots%20%5Cright%29&plus;i%5Cleft%28x-%7B%5Cfrac%20%7Bx%5E%7B3%7D%7D%7B3%21%7D%7D&plus;%7B%5Cfrac%20%7Bx%5E%7B5%7D%7D%7B5%21%7D%7D-%7B%5Cfrac%20%7Bx%5E%7B7%7D%7D%7B7%21%7D%7D&plus;%5Ccdots%20%5Cright%29%5C%5C%5B8pt%5D%26%3D%5Ccos%20x&plus;i%5Csin%20x%2C%5Cend%7Baligned%7D%7D%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle {\begin{aligned}e^{ix}&=1+ix+{\frac {(ix)^{2}}{2!}}+{\frac {(ix)^{3}}{3!}}+{\frac {(ix)^{4}}{4!}}+{\frac {(ix)^{5}}{5!}}+{\frac {(ix)^{6}}{6!}}+{\frac {(ix)^{7}}{7!}}+{\frac {(ix)^{8}}{8!}}+\cdots \\[8pt]&=1+ix-{\frac {x^{2}}{2!}}-{\frac {ix^{3}}{3!}}+{\frac {x^{4}}{4!}}+{\frac {ix^{5}}{5!}}-{\frac {x^{6}}{6!}}-{\frac {ix^{7}}{7!}}+{\frac {x^{8}}{8!}}+\cdots \\[8pt]&=\left(1-{\frac {x^{2}}{2!}}+{\frac {x^{4}}{4!}}-{\frac {x^{6}}{6!}}+{\frac {x^{8}}{8!}}-\cdots \right)+i\left(x-{\frac {x^{3}}{3!}}+{\frac {x^{5}}{5!}}-{\frac {x^{7}}{7!}}+\cdots \right)\\[8pt]&=\cos x+i\sin x,\end{aligned}}}" />



Letting <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%3D%5Cmathbf%7Bu%7D%5Ctheta" alt="https://latex.codecogs.com/svg.latex?\mathbf{v}=\mathbf{u}\theta" />  be the rotation vector, representing a rotation of <img src="https://latex.codecogs.com/svg.latex?%5Ctheta" alt="https://latex.codecogs.com/svg.latex?\theta" />
 radians around the unitary axis 

<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bu%7D%3D%5Cbegin%7Bbmatrix%7Du_x%20%5C%5C%20u_y%20%5C%5C%20u_z%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{u}=\begin{bmatrix}u_x \\ u_y \\ u_z\end{bmatrix}" />
 
  
we can get its exponential series as:





<img src="https://latex.codecogs.com/svg.latex?e%5E%5Cmathbf%7Bv%7D%20%3D%20e%5E%7B%5Cmathbf%7Bu%7D%5Ctheta%7D%20%3D%20%5CBig%281%20-%20%5Cfrac%7B%5Ctheta%5E2%7D%7B2%21%7D%20&plus;%20%5Cfrac%7B%5Ctheta%5E4%7D%7B4%21%7D%20&plus;%20%5Ccdots%5CBig%29%20&plus;%20%5CBig%28%5Cmathbf%7Bu%7D%5Ctheta%20-%20%5Cfrac%7B%5Cmathbf%7Bu%7D%5Ctheta%5E3%7D%7B3%21%7D%20&plus;%20%5Cfrac%7B%5Cmathbf%7Bu%7D%5Ctheta%5E5%7D%7B5%21%7D%20&plus;%20%5Ccdots%5CBig%29" alt="https://latex.codecogs.com/svg.latex?e^\mathbf{v} = e^{\mathbf{u}\theta} = \Big(1 - \frac{\theta^2}{2!} + \frac{\theta^4}{4!} + \cdots\Big) + \Big(\mathbf{u}\theta - \frac{\mathbf{u}\theta^3}{3!} + \frac{\mathbf{u}\theta^5}{5!} + \cdots\Big)" />


This exponential map is formerly defined as:




<img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bq%7D%20%3D%20e%5E%5Cmathbf%7Bv%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%5Ccos%5Cfrac%7B%5Ctheta%7D%7B2%7D%20%5C%5C%20%5Cmathbf%7Bu%7D%5Csin%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{q} = e^\mathbf{v} = \begin{bmatrix}\cos\frac{\theta}{2} \\ \mathbf{u}\sin\frac{\theta}{2}\end{bmatrix}" />








# IMU Integration

Refs: [1](https://rpg.ifi.uzh.ch/docs/teaching/2018/13_visual_inertial_fusion_advanced.pdf), [2](http://mars.cs.umn.edu/tr/Stergios_VINS_Tutorial_IROS19.pdf)


# Noise Spectral Density


# Signal-to-noise Ratio




# Allan Variance curve

Allan variance plot (also known as the two-sample variance plot or the Hadamard variance plot) is a graphical tool used in signal processing to analyze the stability of a signal over time

The Allan deviation measures the fluctuations of a signal between two time intervals, and the integration time is the length of the time interval over which the fluctuations are averaged.
By analyzing the slope of the plot, one can determine the dominant sources of noise in the signal and estimate the noise power spectral density.

In general, a flat line on an Allan variance plot indicates white noise, while a slope of -1/2 indicates flicker noise, and a slope of -1 indicates random walk noise. These noise types are commonly found in electronic systems, such as oscillators and clocks, and the Allan variance plot can help engineers optimize the design of these systems for their intended applications.

## Variance


<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%5Coperatorname%20%7BVar%7D%20%28X%29%3D%7B%5Cfrac%20%7B1%7D%7Bn%7D%7D%5Csum%20_%7Bi%3D1%7D%5E%7Bn%7D%28x_%7Bi%7D-%5Cmu%20%29%5E%7B2%7D%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle \operatorname {Var} (X)={\frac {1}{n}}\sum _{i=1}^{n}(x_{i}-\mu )^{2}}" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%5Cmu%20%3D%7B%5Cfrac%20%7B1%7D%7Bn%7D%7D%5Csum%20_%7Bi%3D1%7D%5E%7Bn%7Dx_%7Bi%7D.%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle \mu ={\frac {1}{n}}\sum _{i=1}^{n}x_{i}.}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?%5Coperatorname%20%7BVar%7D%20%28X%29%3D%7B%5Cfrac%20%7B1%7D%7Bn%5E%7B2%7D%7D%7D%5Csum%20_%7Bi%3D1%7D%5E%7Bn%7D%5Csum%20_%7Bj%3D1%7D%5E%7Bn%7D%7B%5Cfrac%20%7B1%7D%7B2%7D%7D%28x_%7Bi%7D-x_%7Bj%7D%29%5E%7B2%7D%3D%7B%5Cfrac%20%7B1%7D%7Bn%5E%7B2%7D%7D%7D%5Csum%20_%7Bi%7D%5Csum%20_%7Bj%3Ei%7D%28x_%7Bi%7D-x_%7Bj%7D%29%5E%7B2%7D." alt="https://latex.codecogs.com/svg.latex?\operatorname {Var} (X)={\frac {1}{n^{2}}}\sum _{i=1}^{n}\sum _{j=1}^{n}{\frac {1}{2}}(x_{i}-x_{j})^{2}={\frac {1}{n^{2}}}\sum _{i}\sum _{j>i}(x_{i}-x_{j})^{2}." />


## M-sample variance



<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%5Csigma%20_%7By%7D%5E%7B2%7D%28M%2CT%2C%5Ctau%20%29%3D%7B%5Cfrac%20%7B1%7D%7BM-1%7D%7D%5Cleft%5C%7B%5Csum%20_%7Bi%3D0%7D%5E%7BM-1%7D%7B%5Cbar%20%7By%7D%7D_%7Bi%7D%5E%7B2%7D-%7B%5Cfrac%20%7B1%7D%7BM%7D%7D%5Cleft%5B%5Csum%20_%7Bi%3D0%7D%5E%7BM-1%7D%7B%5Cbar%20%7By%7D%7D_%7Bi%7D%5Cright%5D%5E%7B2%7D%5Cright%5C%7D%2C%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle \sigma _{y}^{2}(M,T,\tau )={\frac {1}{M-1}}\left\{\sum _{i=0}^{M-1}{\bar {y}}_{i}^{2}-{\frac {1}{M}}\left[\sum _{i=0}^{M-1}{\bar {y}}_{i}\right]^{2}\right\},}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%5Csigma%20_%7By%7D%5E%7B2%7D%28M%2CT%2C%5Ctau%20%29%3D%7B%5Cfrac%20%7B1%7D%7BM-1%7D%7D%5Cleft%5C%7B%5Csum%20_%7Bi%3D0%7D%5E%7BM-1%7D%5Cleft%5B%7B%5Cfrac%20%7Bx%28iT&plus;%5Ctau%20%29-x%28iT%29%7D%7B%5Ctau%20%7D%7D%5Cright%5D%5E%7B2%7D-%7B%5Cfrac%20%7B1%7D%7BM%7D%7D%5Cleft%5B%5Csum%20_%7Bi%3D0%7D%5E%7BM-1%7D%7B%5Cfrac%20%7Bx%28iT&plus;%5Ctau%20%29-x%28iT%29%7D%7B%5Ctau%20%7D%7D%5Cright%5D%5E%7B2%7D%5Cright%5C%7D%2C%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle \sigma _{y}^{2}(M,T,\tau )={\frac {1}{M-1}}\left\{\sum _{i=0}^{M-1}\left[{\frac {x(iT+\tau )-x(iT)}{\tau }}\right]^{2}-{\frac {1}{M}}\left[\sum _{i=0}^{M-1}{\frac {x(iT+\tau )-x(iT)}{\tau }}\right]^{2}\right\},}" />


<br/>
<br/>

Gyro noise Allan variance is calculated by measuring the difference in rotation or angular velocity measurements taken at different times, and then averaging those differences over different time intervals. The result is a plot of the Allan deviation versus the averaging time, 

The Allan deviation is defined as the square root of the variance of the difference between two rotation or angular velocity measurements taken at time intervals `τ` apart, divided by the mean value of the measurements. It is often expressed in units of degrees per hour or radians per second.



## Allan Variance

The Allan variance is defined as:

<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%5Csigma%20_%7By%7D%5E%7B2%7D%28%5Ctau%20%29%3D%5Cleft%5Clangle%20%5Csigma%20_%7By%7D%5E%7B2%7D%282%2C%5Ctau%20%2C%5Ctau%20%29%5Cright%5Crangle%20%2C%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle \sigma _{y}^{2}(\tau )=\left\langle \sigma _{y}^{2}(2,\tau ,\tau )\right\rangle ,}" />


# Autoregressive model

# Madgwick Orientation Filter

Refs: [1](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html)


# Mahony Orientation Filter

Refs: [1](https://ahrs.readthedocs.io/en/latest/filters/mahony.html)

# Simulating IMU Measurements
Refs [1](https://ch.mathworks.com/help/fusion/ug/introduction-to-simulating-imu-measurements.html), [2](https://ch.mathworks.com/help/nav/ref/imu.html)


# IMU Propagation Derivations

Refs: [1](https://docs.openvins.com/propagation.html)


## The IMU Noise Model

The IMU measurement model used in Kalibr contains two types of sensor errors: <img src="https://latex.codecogs.com/svg.latex?n"  alt="https://latex.codecogs.com/svg.latex?n"/>, an additive noise term that fluctuates very rapidly ("white noise"), and <img src="https://latex.codecogs.com/svg.latex?b"  alt="https://latex.codecogs.com/svg.latex?b"/>, a slowly varying sensor bias. The angular rate measurement <img src="https://latex.codecogs.com/svg.latex?\tilde\omega"  alt="https://latex.codecogs.com/svg.latex?\tilde\omega"/> (for one single axis of the gyro, in this case) is therefore written as:

<img src="https://latex.codecogs.com/svg.latex?%5Ctilde%5Comega%28t%29%3D%5Comega%28t%29&plus;b%28t%29&plus;n%28t%29"  alt="https://latex.codecogs.com/svg.latex?\tilde\omega(t)=\omega(t)+b(t)+n(t)"/>

The same model is independently used to model all three sensor axes. The same model (with different parameters, as we will later see) is also used to model the accelerometer measurement errors (on each axis independently). This model is tractable and often used to model inertial sensors.





### Additive "White Noise"

The rapid fluctuations in the sensor signal are modelled heuristically with a zero-mean, independent, continuous-time white Gaussian noise process <img src="https://latex.codecogs.com/svg.latex?n(t)"  alt="https://latex.codecogs.com/svg.latex?n(t)"/> of strength <img src="https://latex.codecogs.com/svg.latex?\sigma_g"  alt="https://latex.codecogs.com/svg.latex?\sigma_g"/>:


<img src="https://latex.codecogs.com/svg.latex?E[n(t)]\equiv0"  alt="https://latex.codecogs.com/svg.latex?E[n(t)]\equiv0"/>

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?E[n(t_1)n(t_2)]=\sigma_g^2\delta(t_1-t_2)"  alt="https://latex.codecogs.com/svg.latex?E[n(t_1)n(t_2)]=\sigma_g^2\delta(t_1-t_2)"/>


<br/>
<br/>

In other words, the higher <img src="https://latex.codecogs.com/svg.latex?\sigma_g"  alt="https://latex.codecogs.com/svg.latex?\sigma_g"/> is, the more "noisy" your gyro measurements. The parameters `gyroscope_noise_density` and `accelerometer_noise_density` in the YAML file specify exactly these noise strengths <img src="https://latex.codecogs.com/svg.latex?\sigma_g"  alt="https://latex.codecogs.com/svg.latex?\sigma_g"/> (gyro) and <img src="https://latex.codecogs.com/svg.latex?\sigma_a"  alt="https://latex.codecogs.com/svg.latex?\sigma_a"/> (accel) for the continuous-time model.

This process can be simulated in **discrete-time** as follows:
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?n_d[k]=\sigma_{g_d}w[k]"  alt="https://latex.codecogs.com/svg.latex?n_d[k]=\sigma_{g_d}w[k]"/>

<br/>
<br/>


with

<br/>

<img src="https://latex.codecogs.com/svg.latex?w[k]\sim\mathcal{N}(0,1)"  alt="https://latex.codecogs.com/svg.latex?w[k]\sim\mathcal{N}(0,1)"/>

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}=\sigma_g\frac{1}{\sqrt{\Delta t}}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{g_d}=\sigma_g\frac{1}{\sqrt{\Delta t}}"/>

where <img src="https://latex.codecogs.com/svg.latex?\Delta t"  alt="https://latex.codecogs.com/svg.latex?\Delta t"/> is the sampling time. This is identical to the discrete-time implementation within Kalibr.

_Note_: This assumes that the noise was filtered with an ideal low-pass filter that filters noise above
<br/>
<br/>

 <img src="https://latex.codecogs.com/svg.latex?f=\frac{1}{2\Delta t}"  alt="https://latex.codecogs.com/svg.latex?f=\frac{1}{2\Delta t}"/>
 
<br/>
<br/>
 
  (in other words, an ideal decimation stage). This may or may not be the case, depending on your sensor settings. If you simply "subsample" your gyro or accel, you are not allowed to scale your "white noise density" in that way.

How you can determine this parameter for your particular IMU is explained below.




### Bias
IMU biases are systematic errors that can be caused by various factors like temperature changes, manufacturing imperfections, or aging. 
In Kalibr, slow variations in the sensor bias are modelled with a "Brownian motion" process, also termed a "Wiener process", or "random walk" in discrete-time. Formally, this process is generated by integrating "white noise" of strength <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"/> (gyro) or <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_a}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_a}"/> (accel):

<img src="https://latex.codecogs.com/svg.latex?\dot{b_g}(t)=\sigma_{b_g}w(t)"  alt="https://latex.codecogs.com/svg.latex?\dot{b_g}(t)=\sigma_{b_g}w(t)"/>

<br/>
<br/>

The formula 
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.latex?\dot{b_g}(t)=\sigma_{b_g}w(t)"  alt="https://latex.codecogs.com/svg.latex?\dot{b_g}(t)=\sigma_{b_g}w(t)"/> 

<br/>
<br/>

represents the rate of change of the gyroscope bias <img src="https://latex.codecogs.com/svg.latex?b_g"  alt="https://latex.codecogs.com/svg.latex?b_g"/> with respect to time <img src="https://latex.codecogs.com/svg.latex?t"  alt="https://latex.codecogs.com/svg.latex?t"/>. Here:
- <img src="https://latex.codecogs.com/svg.latex?\dot{b_g}(t)"  alt="https://latex.codecogs.com/svg.latex?\dot{b_g}(t)"/> denotes the derivative of the bias <img src="https://latex.codecogs.com/svg.latex?b_g"  alt="https://latex.codecogs.com/svg.latex?b_g"/> at time <img src="https://latex.codecogs.com/svg.latex?t"  alt="https://latex.codecogs.com/svg.latex?t"/>.
- <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"/> is the standard deviation of the gyroscope bias noise.
- <img src="https://latex.codecogs.com/svg.latex?w(t)"  alt="https://latex.codecogs.com/svg.latex?w(t)"/> is a white noise process at time <img src="https://latex.codecogs.com/svg.latex?t"  alt="https://latex.codecogs.com/svg.latex?t"/>.

This equation models the gyroscope bias as a stochastic process, where the rate of change of the bias is driven by a white noise process scaled by a standard deviation parameter <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"/>.

where <img src="https://latex.codecogs.com/svg.latex?w"  alt="https://latex.codecogs.com/svg.latex?w"/> is "white noise" of unit strength.

The parameters `gyroscope_random_walk` and `accelerometer_random_walk` in the YAML file specify these noise strengths <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"/> and <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_a}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_a}"/>. The higher the bias variations in your gyro or accel are, the higher these parameters need to be set.

This process can be simulated in discrete-time as follows:

<img src="https://latex.codecogs.com/svg.latex?b_d[k]=b_d[k-1]+\sigma_{bgd}%20w[k]"  alt="https://latex.codecogs.com/svg.latex?b_d[k]=b_d[k-1]+\sigma_{bgd}%20w[k]"/>

<br/>
<br/>

with

<br/>


<img src="https://latex.codecogs.com/svg.latex?w[k]\sim\mathcal{N}(0,1)"  alt="https://latex.codecogs.com/svg.latex?w[k]\sim\mathcal{N}(0,1)"/>


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?\sigma_{bgd}=\sigma_{b_g}\sqrt{\Delta%20t}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{bgd}=\sigma_{b_g}\sqrt{\Delta%20t}"/>

This corresponds to the implementation in Kalibr.

The formula following the mentioned line models the bias of an IMU sensor as a random walk process, by integrating a white noise. In this formula:
- <img src="https://latex.codecogs.com/svg.latex?b[k]"  alt="https://latex.codecogs.com/svg.latex?b[k]"/> represents the bias at time <img src="https://latex.codecogs.com/svg.latex?k"  alt="https://latex.codecogs.com/svg.latex?k"/>.
- <img src="https://latex.codecogs.com/svg.latex?b[k-1]"  alt="https://latex.codecogs.com/svg.latex?b[k-1]"/> is the bias at the previous time step <img src="https://latex.codecogs.com/svg.latex?k-1"  alt="https://latex.codecogs.com/svg.latex?k-1"/>.
- <img src="https://latex.codecogs.com/svg.latex?w[k]"  alt="https://latex.codecogs.com/svg.latex?w[k]"/> denotes white noise at time <img src="https://latex.codecogs.com/svg.latex?k"  alt="https://latex.codecogs.com/svg.latex?k"/>.
- <img src="https://latex.codecogs.com/svg.latex?\sigma_{bgd}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{bgd}"/> is the white noise strength, determining the rate at which the bias changes over time.

This model suggests that the bias at each time step is the sum of the bias at the previous time step and a white noise term, which is scaled by the white noise strength <img src="https://latex.codecogs.com/svg.latex?\sigma_{bgd}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{bgd}"/>.






### The Noise Model Parameters in Kalibr

In Kalibr, the sensor errors are modelled for each sensor axis independently. Unless you are using inertial sensors whose axes have very different noise properties, this is legitimate. Internally, Kalibr will therefore model the "white" noise processes as follows:

<img src="https://latex.codecogs.com/svg.latex?E[\mathbf{n}(t)]=\mathbf{0}_{3\times1}"  alt="https://latex.codecogs.com/svg.latex?E[\mathbf{n}(t)]=\mathbf{0}_{3\times1}"/>

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?E[\mathbf{n}(t_1)\mathbf{n}^T(t_2)]=\sigma_g^2\mathbf{I}_{3\times3}\delta(t_1-t_2)"  alt="https://latex.codecogs.com/svg.latex?E[\mathbf{n}(t_1)\mathbf{n}^T(t_2)]=\sigma_g^2\mathbf{I}_{3\times3}\delta(t_1-t_2)"/>

The bias variations ("random walks") are also modelled independently on each sensor axis. Table 1 summarizes all the parameters, and links them to the entries that you can specify in the YAML file.




* * *

Table 1: Summary of the IMU noise model parameters as they can be specified in the IMU configuration YAML file of Kalibr.
A discussion of these units can be found [in this thread](https://github.com/ethz-asl/kalibr/issues/354#issuecomment-979934812) for those interested.

Parameter | YAML element | Symbol | Units
--- | --- | --- | ---
Gyroscope "white noise" | `gyroscope_noise_density` | <img src="https://latex.codecogs.com/svg.latex?\sigma_{g}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{g}"/> | <img src="https://latex.codecogs.com/svg.latex?\frac{rad}{s}\frac{1}{\sqrt{Hz}}"  alt="https://latex.codecogs.com/svg.latex?\frac{rad}{s}\frac{1}{\sqrt{Hz}}"/>
Accelerometer "white noise" | `accelerometer_noise_density` | <img src="https://latex.codecogs.com/svg.latex?\sigma_{a}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{a}"/> | <img src="https://latex.codecogs.com/svg.latex?\frac{m}{s^2}\frac{1}{\sqrt{Hz}}"  alt="https://latex.codecogs.com/svg.latex?\frac{m}{s^2}\frac{1}{\sqrt{Hz}}"/>
Gyroscope "random walk" | `gyroscope_random_walk` | <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_g}"/> | <img src="https://latex.codecogs.com/svg.latex?\frac{rad}{s^2}\frac{1}{\sqrt{Hz}}"  alt="https://latex.codecogs.com/svg.latex?\frac{rad}{s^2}\frac{1}{\sqrt{Hz}}"/>
Accelerometer "random walk" | `accelerometer_random_walk` | <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_a}"  alt="https://latex.codecogs.com/svg.latex?\sigma_{b_a}"/> | <img src="https://latex.codecogs.com/svg.latex?\frac{m}{s^3}\frac{1}{\sqrt{Hz}}"  alt="https://latex.codecogs.com/svg.latex?\frac{m}{s^3}\frac{1}{\sqrt{Hz}}"/>
IMU sampling rate | `update_rate` | <img src="https://latex.codecogs.com/svg.latex?\frac{1}{\Delta%20t}"  alt="https://latex.codecogs.com/svg.latex?\frac{1}{\Delta%20t}"/> | <img src="https://latex.codecogs.com/svg.latex?Hz"  alt="https://latex.codecogs.com/svg.latex?Hz"/>

* * *

## How to Obtain the Parameters for your IMU

This section describes how you can obtain the Kalibr IMU noise model parameters for your particular IMU. While there are many methods available, we focus on how to get the parameters from the datasheet or using the "Allan standard deviation".

### From the Datasheet of the IMU

**White Noise Terms:** The parameters for the "white noise" processes (<img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g"/>, <img src="https://latex.codecogs.com/svg.latex?\sigma_a" alt="\sigma_a"/>) are often specified in the datasheet of the sensor manufacturer. A bit misleading, they are commonly denoted as **angular random walk** in case of the gyro, and **velocity random walk** for the accel. The name comes from the fact that if this white noise on rate or acceleration is integrated (in the navigation equations), it becomes a "random walk" on the angle or the velocity.

Other manufacturers specify it directly as **rate noise density**, **acceleration noise density**, or simply **noise density**. The name comes from the fact that <img src="https://latex.codecogs.com/svg.latex?\sigma_g^2" alt="\sigma_g^2"/> corresponds to the power spectral _density_ of <img src="https://latex.codecogs.com/svg.latex?n" alt="n"/>. Using the discrete-time implementation outlined above, you can check if you interpreted the values in the datasheet correctly.

**Bias Terms:** In contrast to the "white noise sigmas", <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}" alt="\sigma_{b_g}"/> and <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_a}" alt="\sigma_{b_a}"/> are rarely directly specified in the datasheet. The reason is that in practice, the bias does not truly behave like a "random walk" for longer integration times. Often, the so-called **in-run bias (in)stability** is specified instead. This sensor parameter indicates (approximately) the accuracy with which the bias can be determined (if a random process is the sum of "white noise" and a "random walk" bias, the bias cannot be estimated with arbitrarily low uncertainty at any point in time). In combination with the strength of the "white noise", however, one can often use the in-run bias stability (the lowest point in the Allan standard deviation, see below) to determine reasonable values for <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}" alt="\sigma_{b_g}"/> and <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_a}" alt="\sigma_{b_a}"/> (assuming that the noise is dominated by "white noise" and a "random walk").


### From the Allan standard deviation (AD)

While many (parametric and non-parametric) methods have been proposed to (automatically) determine the noise model parameters from samples, deriving the parameters from an Allan standard deviation plot is probably the most common and standardized procedure. It derives the AD for different random processes, including "white noise" (slope -1/2 in a log-log AD plot) and "random walk" (slope +1/2 in a log-log AD plot) that are used in Kalibr. The noise model parameters can be determined directly from the Allan standard deviation.

<img src="https://latex.codecogs.com/svg.latex?\sigma_{g}" alt="\sigma_{g}"/> and <img src="https://latex.codecogs.com/svg.latex?\sigma_{a}" alt="\sigma_{a}"/> correspond to the values at \(AD(\tau=1s)\) (point (1) in the figure below). This is only true since the noise power in most inertial sensors is dominated by "white noise" at a frequency of approximately 1Hz.

<img src="https://latex.codecogs.com/svg.latex?\sigma_{b_g}" alt="\sigma_{b_g}"/> and <img src="https://latex.codecogs.com/svg.latex?\sigma_{b_a}" alt="\sigma_{b_a}"/> are identified as the value of the (fitted) "random walk" diagonal at an integration time of \(AD(\tau=3s)\) (point (2) in the figure below). This can be seen immediately when the Allan standard deviation is derived for a "random walk" process.

---

**Figure 1: Allan standard deviation of a MEMS gyro with manually identified noise processes.**

<img src="https://cloud.githubusercontent.com/assets/1916839/3589506/8f57d0ee-0c4e-11e4-9ab4-33821c040490.png" width="85%" align="middle"/>


## Kalibr IMU Noise Parameters in Practice

Some manufacturers provide an Allan standard deviation plot in the datasheet of the device. Otherwise, it needs to be computed from sensor data.
A useful open source tool for computing IMU parameters using Allan Deviation is [ori-drs/allan_variance_ros](https://github.com/ori-drs/allan_variance_ros) which seems to be actively maintained.
We recommend using this tool directly on ~15-24 hour dataset recording of the IMU being stationary.


_It is important_ to note that the IMU measurement error model used here is derived from a sensor which does not undergo motion, and at constant temperature. Hence scale factor errors and bias variation caused by temperature changes, for example, are not accounted for. So clearly, the model is optimistic. Particularly when using low-cost MEMS IMUs with Kalibr, you may have to increase the noise model parameters to "capture" these errors as well. In other words, if you use directly the "sigmas" obtained from static sensor data, Kalibr will tend to trust your IMU measurements too much, and its solution will not be optimal.

From our experience, for lowest-cost sensors, increasing the noise model parameters by a factor of 10x or more may be necessary. If you use Kalibr with such a device, please give us feedback, such that we can develop specific guidelines, device-specific parameter suggestions, or more advanced methods to determine these parameters.

***


# IMU Noise Model

<img src="https://latex.codecogs.com/svg.latex?E[n(t_1)n(t_2)]%20=%20\sigma^2%20\delta(t_1%20-%20t_2)" alt="E[n(t_1)n(t_2)] = \sigma^2 \delta(t_1 - t_2)" />

where <img src="https://latex.codecogs.com/svg.latex?\delta(t_1%20-%20t_2)" alt="\delta(t_1 - t_2)" /> is the Dirac delta function and <img src="https://latex.codecogs.com/svg.latex?\sigma^2" alt="\sigma^2" /> is the variance of the noise process.

The expression represents the autocorrelation function of the additive white noise process. Here's a breakdown of why <img src="https://latex.codecogs.com/svg.latex?E[n(t_1)n(t_2)]" alt="E[n(t_1)n(t_2)]" /> relates to the standard deviation:

1. White noise by definition has a constant power spectrum across all frequencies. In the time domain, this is reflected in its autocorrelation function: the noise values at different times are uncorrelated.

2. The expectation <img src="https://latex.codecogs.com/svg.latex?E[n(t_1)n(t_2)]" alt="E[n(t_1)n(t_2)]" /> quantifies how two noise values at times <img src="https://latex.codecogs.com/svg.latex?t_1" alt="t_1" /> and <img src="https://latex.codecogs.com/svg.latex?t_2" alt="t_2" /> are related. If the noise is white (i.e., purely random and uncorrelated), then the only non-zero expectation value is when <img src="https://latex.codecogs.com/svg.latex?t_1%20=%20t_2" alt="t_1 = t_2" />.

3. The Dirac delta function <img src="https://latex.codecogs.com/svg.latex?\delta(t_1%20-%20t_2)" alt="\delta(t_1 - t_2)" /> is a function that's zero everywhere except at <img src="https://latex.codecogs.com/svg.latex?t_1%20=%20t_2" alt="t_1 = t_2" />, where it's "infinitely high" such that its integral over all time is 1. This ensures that the expectation is only non-zero when the two times are the same.

4. The factor <img src="https://latex.codecogs.com/svg.latex?\sigma^2" alt="\sigma^2" /> in front of the Dirac delta function is the variance of the noise process. The standard deviation of the noise is <img src="https://latex.codecogs.com/svg.latex?\sigma" alt="\sigma" />, so <img src="https://latex.codecogs.com/svg.latex?\sigma^2" alt="\sigma^2" /> is the variance. The reason the variance appears here is that the autocorrelation function at zero time-lag (i.e., <img src="https://latex.codecogs.com/svg.latex?t_1%20=%20t_2" alt="t_1 = t_2" />) represents the variance of the noise.

So, while <img src="https://latex.codecogs.com/svg.latex?E[n(t_1)n(t_2)]" alt="E[n(t_1)n(t_2)]" /> as a whole represents the autocorrelation function, the term <img src="https://latex.codecogs.com/svg.latex?\sigma^2" alt="\sigma^2" /> in the expression is indeed the variance of the noise process, and its square root would be the standard deviation.








---

A discrete-time simulation of additive white noise in an Inertial Measurement Unit (IMU). Here's the breakdown of the equation and the terms involved:

<img src="https://latex.codecogs.com/svg.latex?n_d[k]%20=%20\sigma_{g_d}%20w[k]" alt="n_d[k] = \sigma_{g_d} w[k]" />

Here:
- <img src="https://latex.codecogs.com/svg.latex?n_d[k]" alt="n_d[k]" /> represents the discrete-time white noise at a specific time index \(k\).
- <img src="https://latex.codecogs.com/svg.latex?w[k]" alt="w[k]" /> is a standard normal random variable with a mean of \(0\) and a variance of \(1\), denoted as <img src="https://latex.codecogs.com/svg.latex?w[k]%20\sim%20\mathcal{N}(0,1)" alt="w[k] \sim \mathcal{N}(0,1)" />.
- <img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}" alt="\sigma_{g_d}" /> is the discrete-time noise standard deviation, calculated as <img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}%20=%20\sigma_g%20\frac{1}{\sqrt{\Delta%20t}}" alt="\sigma_{g_d} = \sigma_g \frac{1}{\sqrt{\Delta t}}" />, where <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" /> is the continuous-time noise standard deviation, and <img src="https://latex.codecogs.com/svg.latex?\Delta%20t" alt="\Delta t" /> is the sampling interval.

The subscript \( g \) typically stands for "gyroscope" when discussing noise characteristics in the context of Inertial Measurement Units (IMUs). IMUs consist of a combination of sensors including accelerometers, gyroscopes, and sometimes magnetometers. Each of these sensors has its own noise characteristics. In the equations discussed, <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" /> represents the standard deviation of the continuous-time white noise process associated with the gyroscope measurements. Similarly, <img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}" alt="\sigma_{g_d}" /> represents the standard deviation of the discrete-time white noise process for the gyroscope. This notation helps differentiate the noise characteristics of the gyroscope from those of other sensors within the IMU.

Now, let's dive deeper into the components and related concepts:

1. **Discrete-Time White Noise**: In discrete time, a white noise process is characterized by a sequence of uncorrelated random variables with zero mean and constant variance. This is in contrast to continuous-time white noise, which has issues due to the peculiarities of the continuum. The discrete-time white noise process is denoted by <img src="https://latex.codecogs.com/svg.latex?w[k]" alt="w[k]" /> in the equation and follows a standard normal distribution <img src="https://latex.codecogs.com/svg.latex?w[k]%20\sim%20\mathcal{N}(0,1)" alt="w[k] \sim \mathcal{N}(0,1)" />.

2. **IMU Noise Simulation**: IMUs are prone to various types of noise, including white noise. The simulation of IMU noise, especially in discrete time, helps in understanding and mitigating the effects of noise on sensor readings. This is crucial for applications like navigation and tracking, where precise sensor readings are vital.

3. **Noise Standard Deviation**: The term <img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}" alt="\sigma_{g_d}" /> in the equation represents the standard deviation of the discrete-time noise process. It's computed from the continuous-time noise standard deviation <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" /> and the sampling interval <img src="https://latex.codecogs.com/svg.latex?\Delta%20t" alt="\Delta t" />, as <img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}%20=%20\sigma_g%20\frac{1}{\sqrt{\Delta%20t}}" alt="\sigma_{g_d} = \sigma_g \frac{1}{\sqrt{\Delta t}}" />. This conversion is essential for simulating the continuous-time noise process in a discrete-time setting, aligning with the sampling rate of the IMU.

---

# The standard deviation of the discrete-time noise process

The standard deviation of the discrete-time noise process is derived from the continuous-time noise standard deviation by accounting for the sampling interval. This conversion is essential for simulating the continuous-time noise process in a discrete-time setting.

The formula to compute the standard deviation of the discrete-time noise process (<img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}" alt="\sigma_{g_d}" />) from the continuous-time noise standard deviation (<img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" />) is given by:

<img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}%20=%20\sigma_g%20\cdot%20\frac{1}{\sqrt{\Delta%20t}}" alt="\sigma_{g_d} = \sigma_g \cdot \frac{1}{\sqrt{\Delta t}}" />

where:
- <img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}" alt="\sigma_{g_d}" /> is the standard deviation of the discrete-time noise process.
- <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" /> is the continuous-time noise standard deviation.
- <img src="https://latex.codecogs.com/svg.latex?\Delta%20t" alt="\Delta t" /> is the sampling interval (time between successive samples).

### Example:

Suppose we have a continuous-time noise standard deviation of <img src="https://latex.codecogs.com/svg.latex?\sigma_g%20=%200.2" alt="\sigma_g = 0.2" /> rad/s, and our IMU is sampling at a rate of 100 Hz, which means the sampling interval is <img src="https://latex.codecogs.com/svg.latex?\Delta%20t%20=%201/100%20=%200.01" alt="\Delta t = 1/100 = 0.01" /> seconds. We can use the formula to compute the standard deviation of the discrete-time noise process:

<img src="https://latex.codecogs.com/svg.latex?\sigma_{g_d}%20=%200.2%20\cdot%20\frac{1}{\sqrt{0.01}}%20=%200.2%20\cdot%2010%20=%202" alt="\sigma_{g_d} = 0.2 \cdot \frac{1}{\sqrt{0.01}} = 0.2 \cdot 10 = 2" /> rad/s

```python
import math

# Given values
sigma_g = 0.2  # continuous-time noise standard deviation (rad/s)
sampling_rate = 100  # Hz
delta_t = 1 / sampling_rate  # sampling interval (s)

# Computing the standard deviation of the discrete-time noise process
sigma_g_d = sigma_g / math.sqrt(delta_t)
print(f'The standard deviation of the discrete-time noise process is {sigma_g_d} rad/s')

```

The standard deviation of the discrete-time noise process is 2.0 rad/s

In this code:
1. We first import the `math` module to use the `sqrt` function.
2. We then define the given values: the continuous-time noise standard deviation `sigma_g`, the sampling rate, and the sampling interval `delta_t`.
3. Finally, we compute the standard deviation of the discrete-time noise process `sigma_g_d` using the formula and print the result.


The standard deviation of continuous-time noise for an IMU, denoted as <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" /> for the gyroscope and possibly <img src="https://latex.codecogs.com/svg.latex?\sigma_a" alt="\sigma_a" /> for the accelerometer, is typically obtained through one of the following ways:

1. **Manufacturer Specifications:**
   - IMU manufacturers often provide noise characteristics in their datasheets. This includes the standard deviation or other statistical measures of sensor noise.

2. **Experimental Measurement:**
   - Collect raw data from the IMU in a controlled environment, and compute the standard deviation of the noise.
   - One common method is the Allan Variance analysis, which helps determine various noise characteristics including the standard deviation of the white noise.

3. **Noise Analysis:**
   - By analyzing the sensor output data and comparing it against known references or ground truth, you can compute or estimate the standard deviation of the noise.

### Example:

Suppose you collect a set of gyroscope data over a period, with the IMU kept stationary to minimize motion-induced readings:

```python
import numpy as np

# Assume gyro_data is a numpy array containing your gyroscope readings
gyro_data = np.array([...])  # replace with your data

# Compute the standard deviation
sigma_g = np.std(gyro_data)

print(f'Standard deviation of continuous-time noise: {sigma_g}')
```

In this code snippet:
- We assume `gyro_data` is a numpy array containing your gyroscope readings.
- We use `np.std` to compute the standard deviation of these readings, which gives an estimate of <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" />.

### Note:
- The accuracy of <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" /> obtained through experimental measurement depends on the quality and duration of data collection, as well as the conditions under which the data is collected.
- If possible, it's always a good practice to cross-verify the obtained <img src="https://latex.codecogs.com/svg.latex?\sigma_g" alt="\sigma_g" /> with manufacturer specifications or other reputable sources.

