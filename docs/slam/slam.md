# SLAM






## Multivariate Gaussians

### Moments parameterization
<img src="https://latex.codecogs.com/svg.latex?p%28%5Cmathbf%7Bx%7D%20%29%3D%7B%5Cdisplaystyle%20%282%5Cpi%20%29%5E%7B-k/2%7D%5Cdet%28%7B%5Cboldsymbol%20%7B%5CSigma%20%7D%7D%29%5E%7B-1/2%7D%5C%2C%5Cexp%20%5Cleft%28-%7B%5Cfrac%20%7B1%7D%7B2%7D%7D%28%5Cmathbf%20%7Bx%7D%20-%7B%5Cboldsymbol%20%7B%5Cmu%20%7D%7D%29%5E%7B%5Cmathsf%20%7BT%7D%7D%7B%5Cboldsymbol%20%7B%5CSigma%20%7D%7D%5E%7B-1%7D%28%5Cmathbf%20%7Bx%7D%20-%7B%5Cboldsymbol%20%7B%5Cmu%20%7D%7D%29%5Cright%29%2C%7D" alt="https://latex.codecogs.com/svg.latex?p(\mathbf{x} )={\displaystyle (2\pi )^{-k/2}\det({\boldsymbol {\Sigma }})^{-1/2}\,\exp \left(-{\frac {1}{2}}(\mathbf {x} -{\boldsymbol {\mu }})^{\mathsf {T}}{\boldsymbol {\Sigma }}^{-1}(\mathbf {x} -{\boldsymbol {\mu }})\right)}" />

## Canonical Parameterization
Alternative representation for Gaussians

<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20%5COmega%20%3D%5CSigma%5E%7B-1%7D%20%5C%5C%20%5Cxi%20%3D%5CSigma%5E%7B-1%7D%5Cmu" alt="https://latex.codecogs.com/svg.latex?\\
\Omega =\Sigma^{-1}
\\
\xi =\Sigma^{-1}\mu" />


<img src="https://latex.codecogs.com/svg.latex?p%28%5Cmathbf%7Bx%7D%20%29%3D%5Cfrac%7Bexp%28-%5Cfrac%7B1%7D%7B2%7D%5Cmu%5ET%5Cxi%20%29%7D%7Bdet%282%5Cpi%5COmega%5E%7B-1%7D%29%5E%7B%5Cfrac%7B1%7D%7B2%7D%7D%20%7Dexp%28-%5Cfrac%7B1%7D%7B2%7D%5Cmathbf%7Bx%7D%5ET%5COmega%20%5Cmathbf%7Bx%7D&plus;%5Cmathbf%7Bx%7D%5ET%5Cxi%20%29" alt="https://latex.codecogs.com/svg.latex?p(\mathbf{x} )=\frac{exp(-\frac{1}{2}\mu^T\xi )}{det(2\pi\Omega^{-1})^{\frac{1}{2}} }exp(-\frac{1}{2}\mathbf{x}^T\Omega \mathbf{x}+\mathbf{x}^T\xi  )" />


<br/>

<img src="images/towards_the_information_form.jpg" alt="images/towards_the_information_form.jpg" width= "50%"  height= "50%" />

<br/>

## Further reading


- simple_2d_slam
Refs: [1](https://github.com/goldbattle/simple_2d_slam)

- Robust Pose-graph Optimization
Refs: [1](https://www.youtube.com/watch?v=zOr9HreMthY)

- Awesome Visual Odometry
Refs: [1](https://github.com/chinhsuanwu/awesome-visual-odometry)

- Monocular Video Odometry Using OpenCV
Refs: [1](https://github.com/alishobeiri/Monocular-Video-Odometery)

- modern-slam-tutorial-python
Refs: [1](https://github.com/gisbi-kim/modern-slam-tutorial-python)

- Monocular-Video-Odometery
Refs: [1](https://github.com/alishobeiri/Monocular-Video-Odometery/blob/master/monovideoodometery.py)
  


- DBoW2: library for indexing and converting images into a bag-of-word representation  
Refs: [1](https://github.com/dorian3d/DBoW2)








## add apriltag to loop closure

Refs: [1](https://berndpfrommer.github.io/tagslam_web/)



## DROID-SLAM

## Hierarchical-Localization

## image-matching-webui

## LightGlue


## DenseSFM

Refs: [1](https://github.com/tsattler/visuallocalizationbenchmark)

## Pixel-Perfect Structure-from-Motion
Refs: [1](https://github.com/cvg/pixel-perfect-sfm)

## ODM
```
docker run -ti --rm -v /home/$USER/workspace/odm_projects/datasets/code/:/datasets/code opendronemap/odm --project-path /datasets
```
[Datasets](https://www.opendronemap.org/odm/datasets/)


## AnyLoc: Towards Universal Visual Place Recognition

Refs [1](https://github.com/AnyLoc/AnyLoc)


