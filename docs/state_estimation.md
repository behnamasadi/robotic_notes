# Bayes filter 


<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20%5Ctext%7B1.%20%7D%20P%28A%7CB%29%3D%5Cfrac%7BP%28A%2CB%29%7D%7BP%28B%29%7D%20%5C%5C%20P%28x%29%3D%5Cint%20P%28x%2Cy%29dy%20%3D%5Csum%20P%28x%7Cy%29P%28y%29%20%5C%5C%20P%28x%29%3D%5Csum%20P%28x%2Cy_i%29%20%5C%5C%20%5Ctext%7B2.%20%7DP%28A%7CB%2CC%29%3D%5Cfrac%7BP%28A%2CB%2CC%29%7D%20%7BP%28B%2CC%29%7D%3D%5Cfrac%7BP%28B%7CA%2CC%29P%28A%2CC%29%7D%7BP%28B%7CC%29P%28C%29%7D%3D%5Cfrac%7BP%28B%7CA%2CC%29P%28A%7CC%29P%28C%29%7D%7BP%28B%7CC%29P%28C%29%7D%3D%5Cfrac%7BP%28B%7CA%2CC%29P%28A%7CC%29%7D%7BP%28B%7CC%29%7D" alt="https://latex.codecogs.com/svg.latex?\\
\text{1. } P(A|B)=\frac{P(A,B)}{P(B)} \\ P(x)=\int P(x,y)dy =\sum P(x|y)P(y) \\ P(x)=\sum P(x,y_i) \\ \text{2. }P(A|B,C)=\frac{P(A,B,C)} {P(B,C)}=\frac{P(B|A,C)P(A,C)}{P(B|C)P(C)}=\frac{P(B|A,C)P(A|C)P(C)}{P(B|C)P(C)}=\frac{P(B|A,C)P(A|C)}{P(B|C)} " />

<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20%5Ctext%7B3.%20%7DP%28A%2CB%29%3D%5Csum%20P%28A%2CB%2CC_i%29%3D%5Csum%20P%28A%7CB%2CC_i%29P%28B%2CC_i%29%3D%5Csum%20P%28A%7CB%2CC_i%29P%28C_i%7CB%29P%28B%29%20%5C%5C%20%5Ctext%7B4.%20%7D%20P%28A%7CB%29%3D%5Cfrac%7BP%28A%2CB%29%7D%7BP%28B%29%7D%3D%5Cfrac%7B%5Csum%20P%28A%7CB%2CC_i%29P%28C_i%7CB%29P%28B%29%7D%7BP%28B%29%7D%20%3D%5Csum%20P%28A%7CB%2CC_i%29P%28C_i%7CB%29" alt="https://latex.codecogs.com/svg.latex?\\
\text{3. }P(A,B)=\sum P(A,B,C_i)=\sum P(A|B,C_i)P(B,C_i)=\sum P(A|B,C_i)P(C_i|B)P(B) \\ \text{4. } P(A|B)=\frac{P(A,B)}{P(B)}=\frac{\sum P(A|B,C_i)P(C_i|B)P(B)}{P(B)} =\sum P(A|B,C_i)P(C_i|B)" />


### Markov property:


<br/>
<br/>
<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20%5Ctext%7B5.%20%7D%20P%28Z_t%7CX_%7B0%3At%7D%2CZ_%7B0%3At-1%7D%2C%20U_%7B1%3At%7D%29%3DP%28Z_t%7CX_t%29%20%5C%5C%20%5Ctext%7B6.%20%7D%20P%28X_t%7CX_%7B1%3At-1%7D%2CZ_%7B1%3At-1%7D%2C%20U_%7B1%3At%7D%29%3DP%28X_t%7CU_t%29" alt="https://latex.codecogs.com/svg.latex?\\ \text{5. } P(Z_t|X_{0:t},Z_{0:t-1}, U_{1:t})=P(Z_t|X_t) \\ \text{6. } P(X_t|X_{1:t-1},Z_{1:t-1}, U_{1:t})=P(X_t|U_t)" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?Bel(x_t)=\eta&space;P(z_t|x_t)\int&space;P(x_t|U_{t-1},x_{t-1})&space;Bel(x_{t-1})dx_{t-1}" title="https://latex.codecogs.com/svg.image?Bel(x_t)=\eta P(z_t|x_t)\int P(x_t|U_{t-1},x_{t-1}) Bel(x_{t-1})dx_{t-1}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?Bel%28X_t%29%3DP%28%5Cunderbrace%7BX_t%7D_A%7C%5Cunderbrace%7BU_1%2C%20Z_1%2C...%2CU_t%7D_C%20%2C%20%5Cunderbrace%7BZ_t%7D_B%29" alt="https://latex.codecogs.com/svg.latex?Bel(X_t)=P(\underbrace{X_t}_A|\underbrace{U_1, Z_1,...,U_t}_C , \underbrace{Z_t}_B)"  />
<br/>
<br/>
Using (2) we will have:

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.latex?Bel%28X_t%29%3DP%28%5Cunderbrace%7BX_t%7D_A%7C%5Cunderbrace%7BU_1%2C%20Z_1%2C...%2CU_t%7D_C%20%2C%20%5Cunderbrace%7BZ_t%7D_B%29%3D%5Cfrac%7BP%28B%7CA%2CC%29P%28A%7CC%29%7D%7BP%28B%7CC%29%7D%3D%5Ceta%20P%28B%7CA%2CC%29P%28A%7CC%29%3D%5Ceta%20P%28Z_t%7CX_t%2CU_1%2C%20Z_1%2C...%2CU_t%29P%28X_t%7CU_1%2C%20Z_1%2C...%2CU_t%29" alt="https://latex.codecogs.com/svg.latex?Bel(X_t)=P(\underbrace{X_t}_A|\underbrace{U_1, Z_1,...,U_t}_C , \underbrace{Z_t}_B)=\frac{P(B|A,C)P(A|C)}{P(B|C)}=\eta P(B|A,C)P(A|C)=\eta P(Z_t|X_t,U_1, Z_1,...,U_t)P(X_t|U_1, Z_1,...,U_t)" />

now using (5),


<img src="https://latex.codecogs.com/svg.latex?Bel%28X_t%29%3D%5Ceta%20P%28Z_t%7CX_t%29P%28X_t%7CU_1%2C%20Z_1%2C...%2CU_t%29" alt="https://latex.codecogs.com/svg.latex?Bel(X_t)=\eta P(Z_t|X_t)P(X_t|U_1, Z_1,...,U_t)" />

Now by using (4):

<img src="https://latex.codecogs.com/svg.latex?Bel%28X_t%29%3D%5Ceta%20P%28Z_t%7CX_t%29%5Cint%20P%28X_t%7CU_1%2C%20Z_1%2C...%2CU_t%2C%20X_%7Bt-1%7D%29P%28X_%7Bt-1%7D%7CU_1%2C%20Z_1%2C...%2CU_t%29%20dX_%7Bt-1%7D" alt="https://latex.codecogs.com/svg.latex?Bel(X_t)=\eta P(Z_t|X_t)\int P(X_t|U_1, Z_1,...,U_t, X_{t-1})P(X_{t-1}|U_1, Z_1,...,U_t) dX_{t-1}" />

Now by using (6):

<img src="https://latex.codecogs.com/svg.latex?Bel%28X_t%29%3D%5Ceta%20P%28Z_t%7CX_t%29%5Cint%20P%28X_t%7CU_t%2C%20X_%7Bt-1%7D%29P%28X_%7Bt-1%7D%7CU_1%2C%20Z_1%2C...%2CU_t%29%20dX_%7Bt-1%7D" alt="https://latex.codecogs.com/svg.latex?Bel(X_t)=\eta P(Z_t|X_t)\int P(X_t|U_t, X_{t-1})P(X_{t-1}|U_1, Z_1,...,U_t) dX_{t-1}" />


since <img src="https://latex.codecogs.com/svg.latex?X_t-1" alt="https://latex.codecogs.com/svg.latex?X_t-1" /> has no dependency on <img src="https://latex.codecogs.com/svg.latex?U_t" alt="https://latex.codecogs.com/svg.latex?U_t" />, we can omit it:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.latex?Bel%28X_t%29%3D%5Ceta%20P%28Z_t%7CX_t%29%5Cint%20P%28X_t%7CU_t%2C%20X_%7Bt-1%7D%29P%28X_%7Bt-1%7D%7CU_1%2C%20Z_1%2C...%2CZ_%7Bt-1%7D%2CU_%7Bt-1%7D%29%20dX_%7Bt-1%7D" alt="https://latex.codecogs.com/svg.latex?Bel(X_t)=\eta P(Z_t|X_t)\int P(X_t|U_t, X_{t-1})P(X_{t-1}|U_1, Z_1,...,Z_{t-1},U_{t-1}) dX_{t-1}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?Bel%28X_t%29%3D%5Ceta%20P%28Z_t%7CX_t%29%5Cint%20P%28X_t%7CU_t%2C%20X_%7Bt-1%7D%29Bel%28X_%7Bt-1%7D%29%20dX_%7Bt-1%7D" alt="https://latex.codecogs.com/svg.latex?Bel(X_t)=\eta P(Z_t|X_t)\int P(X_t|U_t, X_{t-1})Bel(X_{t-1}) dX_{t-1}" />



###  Property of  Covariance

<img src="https://latex.codecogs.com/svg.image?Cov(x)=\Sigma&space;" title="https://latex.codecogs.com/svg.image?Cov(x)=\Sigma " />
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?Cov(Ax)=A&space;\Sigma&space;A^T&space;" title="https://latex.codecogs.com/svg.image?Cov(Ax)=A \Sigma A^T " />

<br/>
<br/>

###  Product of Gaussian

<img src="https://latex.codecogs.com/svg.image?\mathcal{N}(\mu_0,\,\sigma_0^{2})&space;\mathcal{N}(\mu_1,\,\sigma_1^{2})=\mathcal{N}(\mu^\prime,\,\sigma^{\prime{2}})&space;" title="https://latex.codecogs.com/svg.image?\mathcal{N}(\mu_0,\,\sigma_0^{2}) \mathcal{N}(\mu_1,\,\sigma_1^{2})=\mathcal{N}(\mu^\prime,\,\sigma^{\prime{2}}) " />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\sigma^{\prime{2}}=\sigma_0^{2}-&space;\frac{\sigma_0^{4}}{\sigma_0^{2}&space;&plus;&space;\sigma_1^{2}}&space;" title="https://latex.codecogs.com/svg.image?\sigma^{\prime{2}}=\sigma_0^{2}- \frac{\sigma_0^{4}}{\sigma_0^{2} + \sigma_1^{2}} " />

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0&plus;&space;\frac{\sigma_0^2(\mu_1-\mu_0)}{\sigma_0^2&plus;\sigma_1^2}" title="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0+ \frac{\sigma_0^2(\mu_1-\mu_0)}{\sigma_0^2+\sigma_1^2}" />

<br/>
<br/>


We can reformulate it as:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?K=\frac{\sigma_0^2}{\sigma_0^2&plus;\sigma_1^2}" title="https://latex.codecogs.com/svg.image?K=\frac{\sigma_0^2}{\sigma_0^2+\sigma_1^2}" />


<img src="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0&plus;K(\mu_1-\mu_0)" title="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0+K(\mu_1-\mu_0)" />
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\sigma^{\prime&space;2}=\sigma_0&space;^{2}-K\sigma_0^2" title="https://latex.codecogs.com/svg.image?\sigma^{\prime 2}=\sigma_0 ^{2}-K\sigma_0^2" />



<br/>
<br/>

For multidimensional data 



<img src="https://latex.codecogs.com/svg.image?K=\Sigma_0(\Sigma_0&space;&plus;\Sigma_1)^{-1}" title="https://latex.codecogs.com/svg.image?K=\Sigma_0(\Sigma_0 +\Sigma_1)^{-1}" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\vec{\mu^\prime}=\vec{\mu_0}&plus;K(\vec{\mu_1}-\vec{\mu_0})" title="https://latex.codecogs.com/svg.image?\vec{\mu^\prime}=\vec{\mu_0}+K(\vec{\mu_1}-\vec{\mu_0})" />

<br/>
<br/>







<img src="https://latex.codecogs.com/svg.image?\Sigma^{\prime&space;}=\Sigma_0&space;^{2}-K\Sigma_0^2" title="https://latex.codecogs.com/svg.image?\Sigma^{\prime }=\Sigma_0 ^{2}-K\Sigma_0^2" />

<br/>
<br/>


###  Newton Law of Motion


<img src="https://latex.codecogs.com/svg.image?x_k=x_{k-1}&plus;v_{k-1}\Delta&space;t&plus;\frac{1}{2}a\Delta&space;t^2" title="https://latex.codecogs.com/svg.image?x_k=x_{k-1}+v_{k-1}\Delta t+\frac{1}{2}a\Delta t^2" />
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?v_k=v_{k-1}&plus;a\Delta&space;t" title="https://latex.codecogs.com/svg.image?v_k=v_{k-1}+a\Delta t" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}x_k&space;\\v_k\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}x_k \\v_k\end{bmatrix}" />

<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}1&space;&&space;\Delta&space;T&space;\\0&space;&&space;&space;1\\\end{bmatrix}\hat{X}_{k-1}&plus;\begin{bmatrix}\frac{\Delta&space;t^2}{2}&space;\\\Delta&space;t\end{bmatrix}a" title="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}1 & \Delta T \\0 & 1\\\end{bmatrix}\hat{X}_{k-1}+\begin{bmatrix}\frac{\Delta t^2}{2} \\\Delta t\end{bmatrix}a" />
<br/>
<br/>

###  Prediction (Estimation)

New mean: 
<br/>
<br/>




<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=F\hat{X}_{k-1}&plus;B\vec{U}" title="https://latex.codecogs.com/svg.image?\hat{X}_k=F\hat{X}_{k-1}+B\vec{U}" />
<br/>
<br/>

New Covariance:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?P_k=F_kP_{k-1}F_k^T&plus;Q_k" title="https://latex.codecogs.com/svg.image?P_k=F_kP_{k-1}F_k^T+Q_k" />


###  Correction (Update)

<img src="https://latex.codecogs.com/svg.image?\mu_k=\underbrace{H_k}_{\text{Measurement&space;Matrix}}&space;\hat{X}_k" title="https://latex.codecogs.com/svg.image?\mu_k=\underbrace{H_k}_{\text{Measurement Matrix}} \hat{X}_k" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\Sigma_k=H_k&space;P_k&space;H_k^T&plus;&space;R_k" title="https://latex.codecogs.com/svg.image?\Sigma_k=H_k P_k H_k^T+ R_k" />

<br/>
<br/>

Fusing and knocking out <img src="https://latex.codecogs.com/svg.image?H_k" title="https://latex.codecogs.com/svg.image?H_k" /> will give us new gain:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?K^\prime=P_K&space;H_K^T(H_k&space;P_k&space;H_k^T&plus;R_k)^{-1}" title="https://latex.codecogs.com/svg.image?K^\prime=P_K H_K^T(H_k P_k H_k^T+R_k)^{-1}" />
<br/>
<br/>
which give us corrected mean:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\hat{X}_k^\prime=\hat{X}_k&space;&plus;&space;K^\prime(Z_k-H_k\hat{X}_k)" title="https://latex.codecogs.com/svg.image?\hat{X}_k^\prime=\hat{X}_k + K^\prime(Z_k-H_k\hat{X}_k)" />


and new covariance:
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?P^\prime&space;_K=P_K-K^\prime&space;H_K&space;P_k" title="https://latex.codecogs.com/svg.image?P^\prime _K=P_K-K^\prime H_K P_k" />

# Extended Kalman Filter


<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" title="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" />

<br/>
<br/>


For instance for a 2D robot with state <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x&space;\\y\\\phi&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x \\y\\\phi \end{bmatrix}" /> and twist command <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;\omega&space;\\V&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} \omega \\V \end{bmatrix}" /> :


<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-\frac{V}{\omega}&space;\sin(\phi)&space;&space;\\-\frac{V}{\omega}&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;\frac{V}{\omega}&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}}_{g(u_t,\mu_{t-1})}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi+\omega \delta t) -\frac{V}{\omega} \sin(\phi) \\-\frac{V}{\omega} \cos(\phi +\omega \delta t) +\frac{V}{\omega} \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}}_{g(u_t,\mu_{t-1})} " />

<br/>
<br/>

First order Linearization:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?f(x)\approx&space;f(a)&plus;&space;\frac{f(a)^\prime}{1!}(x-a)^1" title="https://latex.codecogs.com/svg.image?f(x)\approx f(a)+ \frac{f(a)^\prime}{1!}(x-a)^1" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?g(X_{K-1},U_{K})\approx&space;&space;\frac{\delta&space;g(\mu_K,U_k)}{\delta&space;X_{K-1}}(X_{K-1}-\mu_{K-1})" title="https://latex.codecogs.com/svg.image?g(X_{K-1},U_{K})\approx \frac{\delta g(\mu_K,U_k)}{\delta X_{K-1}}(X_{K-1}-\mu_{K-1})" />


<br/>
<br/>

Linearization of prediction function:
<br/>
<br/>
 
<img src="https://latex.codecogs.com/svg.image?G_K=\frac{\delta&space;g(\mu_K,U_k)}{\delta&space;X_{K-1}}=J=\begin{bmatrix}\frac{\delta&space;g_1}{\delta&space;x_1}&space;&space;&...&space;&space;&\frac{\delta&space;g_1}{\delta&space;x_n}\\\vdots&space;&space;&&space;&\vdots&space;\\&space;&space;\frac{\delta&space;g_m}{\delta&space;x_1}&&space;...&space;&\frac{\delta&space;g_m}{\delta&space;x_n}&space;&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?G_K=\frac{\delta g(\mu_K,U_k)}{\delta X_{K-1}}=J=\begin{bmatrix}\frac{\delta g_1}{\delta x_1} &... &\frac{\delta g_1}{\delta x_n}\\\vdots & &\vdots \\ \frac{\delta g_m}{\delta x_1}& ... &\frac{\delta g_m}{\delta x_n} \end{bmatrix}" />

For example for a 2D mobile robot with laser scanner:

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?Z_k=h(X_k)&plus;\delta&space;k" title="https://latex.codecogs.com/svg.image?Z_k=h(X_k)+\delta k" />



<br/>
<br/>


Linearization of update function:
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?h(X_K)\approx&space;\underbrace{&space;\frac{\partial&space;h(\mu_K)}{\partial&space;X_K}&space;}_H_K&space;(X_K-\mu_K)&space;&plus;h(\mu_k)&space;" title="https://latex.codecogs.com/svg.image?h(X_K)\approx \underbrace{ \frac{\partial h(\mu_K)}{\partial X_K} }_H_K (X_K-\mu_K) +h(\mu_k) " />



## Prediction Step
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" title="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\hat{P_K}=G_K&space;P_{K-1}G_K^T&plus;Q_K" title="https://latex.codecogs.com/svg.image?\hat{P_K}=G_K P_{K-1}G_K^T+Q_K" />
<br/>
<br/>

## Update Step
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?K=\hat{P_K}&space;H_K^T(H_K&space;\hat{P_K}&space;H_K^T&space;&plus;R_K)^{-1}" title="https://latex.codecogs.com/svg.image?K=\hat{P_K} H_K^T(H_K \hat{P_K} H_K^T +R_K)^{-1}" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\hat{X^\prime&space;_K&space;}=\hat{X&space;_K&space;}&plus;K(Z_K&space;-h(\hat{X_K}))" title="https://latex.codecogs.com/svg.image?\hat{X^\prime _K }=\hat{X _K }+K(Z_K -h(\hat{X_K}))" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?P^\prime&space;_K=P_K-K&space;H_K&space;P_K" title="https://latex.codecogs.com/svg.image?P^\prime _K=P_K-K H_K P_K" />




# Finding the appropriate values for Q (process noise covariance) and  R (measurement noise covariance) 
Finding the appropriate values for \( Q \) (process noise covariance) and \( R \) (measurement noise covariance) is crucial for the performance of the Kalman filter. These matrices represent the uncertainties in your model and measurements, respectively.

Here are some methods and guidelines to determine \( Q \) and \( R \):

1. **Physical Insights or Expert Knowledge**:
    - Sometimes, based on the physical understanding of the system, one can approximate the uncertainties in the process and measurements.

2. **Statistical Methods**:
    - If you have access to some ground truth data, you can use it to estimate \( R \). For instance, by comparing your measurements to the ground truth, you can compute the variance or covariance of the measurement error.
    - For \( Q \), if you can measure or estimate the "real" state of the system at multiple time points, you can use these data points to estimate the state transition error, and thus \( Q \).

3. **Tuning**:
    - In the absence of clear physical insight or ample ground truth data, a common approach is to tune the values of \( Q \) and \( R \) empirically to get optimal performance.
    - Start with an initial guess and adjust based on performance. If your estimate is too noisy, you might need to increase \( R \) or decrease \( Q \). If your estimate is lagging behind the real value, you might need to decrease \( R \) or increase \( Q \).
    - This method can be iterative and sometimes time-consuming, but it is often effective.

4. **Process Model**:
    - If you have a physical or mathematical model of the system, you can use it to estimate \( Q \). For example, if you have a motion model and know the variance of acceleration disturbances, you can use that to estimate \( Q \).

5. **Maximum Likelihood Estimation**:
    - If you have a good dataset, you can set up a likelihood function based on the Kalman filter equations and then use optimization techniques to find the values of \( Q \) and \( R \) that maximize this likelihood.

6. **Trial and Error**:
    - Sometimes, simply trying a range of values and evaluating the performance for each can help. This method is brute force and can be computationally intensive but might be effective in some cases.

7. **Use Adaptive Filters**:
    - Adaptive Kalman filters adjust \( Q \) and \( R \) on the fly based on the observed innovations (the difference between estimated measurements and actual measurements). This way, the filter can self-tune to some extent.

Remember that the Kalman filter assumes that the process noise and the measurement noise are Gaussian and white. Ensure that your \( Q \) and \( R \) reflect this assumption, or the filter's performance might be suboptimal.

Lastly, tuning the Kalman filter, especially determining \( Q \) and \( R \), is often as much an art as it is a science. Practical experience, combined with understanding the underlying system, can make the process more intuitive over time.






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


# Error State Extended Kalman Filter


# Invariant extended Kalman filter



# Tracking Multiple Objects

Refs: [1](https://www.youtube.com/watch?v=IIt1LHIHYc4)


# Track-Level Fusion

Refs: [1](https://www.youtube.com/watch?v=r0THmp0WxJI)











## FilterPy
Refs: [1](https://filterpy.readthedocs.io/en/latest/)



# STATE ESTIMATION FOR ROBOTICS

Ref: [1](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf)


