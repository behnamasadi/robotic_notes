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



<img src="https://latex.codecogs.com/svg.latex?=%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D
" alt="https://latex.codecogs.com/svg.latex?=\displaystyle \left[\begin{matrix}- \sin{\left(\theta \right)}\\\sin{\left(\phi \right)} \cos{\left(\theta \right)}\\\cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]" />



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
Noise density refers to the amount of random or unwanted electrical energy that exists within a particular frequency range of a signal. It is usually measured in units of volts per square root of hertz (V/√Hz) or watts per hertz (W/Hz), and it is typically used to describe the level of noise in electronic systems.

In practical terms, noise density is a measure of how much noise is present in a specific frequency band of a signal. For example, if you have a signal that operates at 1 GHz and has a noise density of 10 nV/√Hz, it means that within a 1 Hz bandwidth around the center frequency of 1 GHz, there is 10 nV of noise present.

# Signal-to-noise Ratio
The signal-to-noise ratio (SNR) is a measure of the strength of a signal compared to the level of background noise present in the signal. It is expressed as a ratio of the power of the signal to the power of the noise, often measured in decibels (dB).

A high SNR indicates that the signal is stronger relative to the noise, while a low SNR indicates that the signal is weaker and harder to distinguish from the noise.
In digital image processing, a high SNR means that the image has a high level of detail and is free from visual noise, while a low SNR can result in grainy or blurred images.

To improve the SNR, various techniques can be used such as increasing the signal power, reducing noise sources, using noise filtering or suppression methods, and improving the signal detection or processing algorithms.


# Random Walk


# Angle Random Walk 

# Velocity Random Walk

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

<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%5Csigma%20_%7By%7D%5E%7B2%7D%28%5Ctau%20%29%3D%5Cleft%5Clangle%20%5Csigma%20_%7By%7D%5E%7B2%7D%282%2C%5Ctau%20%2C%5Ctau%20%29%5Cright%5Crangle%20%2C%7D
" alt="https://latex.codecogs.com/svg.latex?{\displaystyle \sigma _{y}^{2}(\tau )=\left\langle \sigma _{y}^{2}(2,\tau ,\tau )\right\rangle ,}" />


# Autoregressive model


