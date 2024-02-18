# GraphSLAM



 <img src="https://latex.codecogs.com/svg.latex?%7B%5E%7Bi%7D_%7Bj%7DX%7D%3D%28%7B%5E%7Bw%7D_%7Bi%7DT%20%5E%7B-1%7D%7D%20%29%5Ctimes%20%28%20%7B%5E%7Bw%7D_%7Bj%7DX%7D%29" alt="https://latex.codecogs.com/svg.latex?{^{i}_{j}X}=({^{w}_{i}T ^{-1}} )\times ( {^{w}_{j}X})" />

<br/>
<br/>

 <img src="https://latex.codecogs.com/svg.latex?%7B%5E%7Bi%7D_%7Bj%7DX%7D%3D%20%5Cbegin%7Bbmatrix%7D%20%7B%5E%7Bw%7D_%7Bi%7DR%7D%5ET%20%26%20-%7B%5E%7Bw%7D_%7Bi%7DR%7D%5ET%20%5Ctimes%20%28%7B%5E%7Bw%7D_%7Bi%7DP%7D%29%20%5C%5C%200%20%26%201%20%5Cend%7Bbmatrix%7D%20%5Ctimes%20%28%20%7B%5E%7Bw%7D_%7Bj%7DX%7D%29" alt="https://latex.codecogs.com/svg.latex?{^{i}_{j}X}= \begin{bmatrix}
{^{w}_{i}R}^T &   -{^{w}_{i}R}^T \times ({^{w}_{i}P})  \\ 
0 & 1  
\end{bmatrix} \times ( {^{w}_{j}X})" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?%7B%5E%7Bi%7D_%7Bj%7DR%7D%3D%28%7B%5E%7Bw%7D_%7Bi%7DR%20%5ET%7D%20%29%5Ctimes%20%28%20%7B%5E%7Bw%7D_%7Bj%7DR%7D%29" alt="https://latex.codecogs.com/svg.latex?{^{i}_{j}R}=({^{w}_{i}R ^T} )\times ( {^{w}_{j}R})" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?%5E%7Bi%7D_jP%3D%5E%7Bw%7D_iR%5ET%20%28%20%7B%5E%7Bw%7D_%7Bj%7DP%7D%20-%7B%5E%7Bw%7D_%7Bi%7DP%7D%20%29" alt="https://latex.codecogs.com/svg.latex?^{i}_jP=^{w}_iR^T (  {^{w}_{j}P} -{^{w}_{i}P} ) " />

which gives us: 



<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%20%5E%7Bi%7D_j%20x%20%5C%5C%20%5E%7Bi%7D_j%20y%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%20cos%28i%5Ew%20%5Ctheta%20%29%20%26%20sin%28i%5Ew%20%5Ctheta%20%29%20%5C%5C%20-sin%28i%5Ew%20%5Ctheta%20%29%20%26%20cos%28i%5Ew%20%5Ctheta%29%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20%5E%7Bw%7D_jx%20-%20%5E%7Bw%7D_ix%20%5C%5C%20%5E%7Bw%7D_jy%20-%20%5E%7Bw%7D_iy%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\begin{bmatrix} ^{i}_j x \\  ^{i}_j y \end{bmatrix}=\begin{bmatrix} cos(i^w \theta ) & sin(i^w \theta ) \\ -sin(i^w \theta ) & cos(i^w \theta) \end{bmatrix} \begin{bmatrix} ^{w}_jx - ^{w}_ix \\  ^{w}_jy - ^{w}_iy \end{bmatrix}" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.latex?%5E%7Bi%7D_j%20%5Ctheta%20%3D%5E%7Bw%7D_j%5Ctheta%20-%7B%5E%7Bw%7D_%7Bi%7D%5Ctheta%7D" alt="https://latex.codecogs.com/svg.latex?^{i}_j \theta =^{w}_j\theta -{^{w}_{i}\theta}" />


our state is:
<br/>

<img src="https://latex.codecogs.com/svg.latex?%5Cmathbb%7BX%7D%3D%28x_1%2Cy_1%2C%5Ctheta_1%2C%20x_2%2Cy_2%2C%20%5Ctheta_2%2C%20...%20%2Cx_n%2Cy_n%2C%5Ctheta_n%29" alt="https://latex.codecogs.com/svg.latex?\mathbb{X}=(x_1,y_1,\theta_1, x_2,y_2, \theta_2, ... ,x_n,y_n,\theta_n)" />

<br/>

our error function is:
<br/>

<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20%5Ctextbf%7Be%7D_%7Bij%7D%28%5Cmathbb%7BX%7D%29%3D%5Ctextbf%7Be%7D_%7Bij%7D%28%5Ctextbf%7BX%7D_%7Bi%7D%2C%5Ctextbf%7BX%7D_%7Bj%7D%29%20%5C%5C%20%5Ctextbf%7BX%7D_%7Bi%7D%3D%28x_i%2Cy_i%2C%5Ctheta_i%29%20%5C%5C%20%5Ctextbf%7BX%7D_%7Bj%7D%3D%28x_j%2Cy_j%2C%5Ctheta_j%29" alt="\\\textbf{e}_{ij}(\mathbb{X})=\textbf{e}_{ij}(\textbf{X}_{i},\textbf{X}_{j})  \\ \textbf{X}_{i}=(x_i,y_i,\theta_i) \\ \textbf{X}_{j}=(x_j,y_j,\theta_j) " />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20%5Ctextbf%7Be%7D_%7Bij%7D%28%5Ctextbf%7BX%7D_%7Bi%7D%2C%5Ctextbf%7BX%7D_%7Bj%7D%29%3D%20%5Cleft%5C%7B%5Cbegin%7Bmatrix%7D%20%5Ei_jx%3Dcos%28%5Ctheta_i%29%28x_j-x_i%29%20&plus;sin%28%5Ctheta_i%29%28y_j-y_i%29%20%5C%5C%20%5Ei_jy%3D-sin%28%5Ctheta_i%29%28x_j-x_i%29&plus;cos%28%5Ctheta_i%29%28y_j-y_i%29%20%5C%5C%20%5E%7Bi%7D_j%20%5Ctheta%20%3D%5E%7Bw%7D_j%5Ctheta%20-%7B%5E%7Bw%7D_%7Bi%7D%5Ctheta%7D%20%5Cend%7Bmatrix%7D%5Cright." alt="https://latex.codecogs.com/svg.latex?\\
\textbf{e}_{ij}(\textbf{X}_{i},\textbf{X}_{j})= \left\{\begin{matrix}  
^i_jx=cos(\theta_i)(x_j-x_i) +sin(\theta_i)(y_j-y_i) \\  ^i_jy=-sin(\theta_i)(x_j-x_i)+cos(\theta_i)(y_j-y_i) \\ ^{i}_j \theta =^{w}_j\theta -{^{w}_{i}\theta} \end{matrix}\right." />



<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20J_%7Bij%7D%3D%5Cbegin%7Bbmatrix%7D%200%20%26%20...%20%26%200%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jx%7D%7B%20%5Cpartial%20_%7Bi%7D%20x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jx%7D%7B%20%5Cpartial%20_%7Bi%7D%20y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%20x%7D%7B%20%5Cpartial%20_%7Bi%7D%20%5Ctheta%7D%20%260%20%26...%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jx%7D%7B%20%5Cpartial%20_%7Bj%7D%20x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jx%7D%7B%20%5Cpartial%20_%7Bj%7D%20y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%20x%7D%7B%20%5Cpartial%20_%7Bj%7D%20%5Ctheta%7D%20%26...%20%260%5C%5C%200%20%26%20...%20%26%200%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jy%7D%7B%20%5Cpartial%20_%7Bi%7D%20x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jy%7D%7B%20%5Cpartial%20_%7Bi%7D%20y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%20y%7D%7B%20%5Cpartial%20_%7Bi%7D%20%5Ctheta%7D%20%260%20%26...%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jy%7D%7B%20%5Cpartial%20_%7Bj%7D%20x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_jy%7D%7B%20%5Cpartial%20_%7Bj%7D%20y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%20y%7D%7B%20%5Cpartial%20_%7Bj%7D%20%5Ctheta%7D%26%20...%20%26%200%5C%5C%200%20%26%20...%20%26%200%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%5Ctheta%7D%7B%20%5Cpartial%20_%7Bi%7D%20x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%5Ctheta%7D%7B%20%5Cpartial%20_%7Bi%7D%20y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%20%5Ctheta%7D%7B%20%5Cpartial%20_%7Bi%7D%20%5Ctheta%7D%20%260%20%26...%26%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%5Ctheta%7D%7B%20%5Cpartial%20_%7Bj%7D%20x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%5Ctheta%7D%7B%20%5Cpartial%20_%7Bj%7D%20y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5E%7Bi%7D_j%20%5Ctheta%7D%7B%20%5Cpartial%20_%7Bj%7D%20%5Ctheta%7D%20%26...%20%26%200%5C%5C%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\\ J_{ij}=\begin{bmatrix} 0 & ... & 0 & \frac{\partial ^{i}_jx}{ \partial _{i} x}      &  \frac{\partial ^{i}_jx}{ \partial _{i} y}  & \frac{\partial ^{i}_j x}{ \partial _{i} \theta} &0 &... & \frac{\partial ^{i}_jx}{ \partial _{j} x}      &  \frac{\partial ^{i}_jx}{ \partial _{j} y}  & \frac{\partial ^{i}_j x}{ \partial _{j} \theta}  &... &0\\ 0 & ... & 0 & \frac{\partial ^{i}_jy}{ \partial _{i} x} &  \frac{\partial ^{i}_jy}{ \partial _{i} y}  & \frac{\partial ^{i}_j y}{ \partial _{i} \theta} &0   &...& \frac{\partial ^{i}_jy}{ \partial _{j} x} &  \frac{\partial ^{i}_jy}{ \partial _{j} y}  & \frac{\partial ^{i}_j y}{ \partial _{j} \theta}& ...     & 0\\ 0 & ... & 0 & \frac{\partial ^{i}_j\theta}{ \partial _{i} x}      &  \frac{\partial ^{i}_j\theta}{ \partial _{i} y}  & \frac{\partial ^{i}_j \theta}{ \partial _{i} \theta} &0 &...&\frac{\partial ^{i}_j\theta}{ \partial _{j} x}      &  \frac{\partial ^{i}_j\theta}{ \partial _{j} y}  & \frac{\partial ^{i}_j \theta}{ \partial _{j} \theta} &...  & 0\\ \end{bmatrix}" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?A_%7Bi%2Cj%7D%3D%5Cbegin%7Bbmatrix%7D%20-cos%28%5Ctheta_i%29%20%26%20-sin%28%5Ctheta_i%29%20%26%20-sin%28%5Ctheta_i%29%28x_j-x_i%29&plus;cos%28%5Ctheta_i%29%28y_j-y_i%29%5C%5C%20sin%28%5Ctheta_i%29%20%26%20-cos%28%5Ctheta_i%29%20%26%20-cos%28%5Ctheta_i%29%28x_j-x_i%29%20-%20sin%28%5Ctheta_i%29%28y_j-y_i%29%5C%5C%200%20%26%200%20%26%20-1%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?A_{i,j}=\begin{bmatrix} -cos(\theta_i) & -sin(\theta_i)  & -sin(\theta_i)(x_j-x_i)+cos(\theta_i)(y_j-y_i)\\ sin(\theta_i) & -cos(\theta_i) & -cos(\theta_i)(x_j-x_i) - sin(\theta_i)(y_j-y_i)\\ 0 & 0 & -1 \end{bmatrix}" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.latex?B_%7Bi%2Cj%7D%3D%5Cbegin%7Bbmatrix%7D%20cos%28%5Ctheta_i%29%20%26%20-sin%28%5Ctheta_i%29%20%26%200%5C%5C%20-sin%28%5Ctheta_i%29%20%26%20cos%28%5Ctheta_i%29%20%26%200%5C%5C%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?B_{i,j}=\begin{bmatrix} cos(\theta_i) & -sin(\theta_i)  & 0\\  -sin(\theta_i) & cos(\theta_i) & 0\\  0 & 0 & 1 \end{bmatrix}" />

<br/>
<br/>




<img src="images/J_i_j.png"  width= "50%"  height= "50%"/>

<br/>
<br/>
<img src="images/b_h.png" width= "50%"  height= "50%"/>

<br/>
<br/>
<img src="images/b_i_j_h_i_j.png" width= "50%"  height= "50%" />

<br/>
<br/>
<img src="images/building_the_linear_system.png" width= "50%"  height= "50%" />





Install the python packages:

```
conda install -c conda-forge opencv
pip install graphslam
conda install conda-forge::gtsam
conda install conda-forge::matplotlib
conda install conda-forge::plotly
```

git submodule update --init



To create a symlink at `destination` which references the original file `<original-ref>`, use:

ln -s <original-ref> <destination>


ln -s ~/workspace/robotic_notes/scripts ~/anaconda3/envs/robotic_notes/



Refs: [1](https://python-graphslam.readthedocs.io/en/stable/), [2](https://github.com/goldbattle/simple_2d_slam)





download g2o examples from [here](https://lucacarlone.mit.edu/datasets/)








