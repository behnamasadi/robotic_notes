## g2o

## Installation

C++ library:

Dependencies:

```
sudo apt install libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
```


```
git clone https://github.com/gabime/spdlog.git
cd spdlog
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/home/$USER/usr -DSPDLOG_BUILD_SHARED=ON
cmake --build build --parallel
cmake --install build
```




```
git clone https://github.com/RainerKuemmerle/g2o/
cd g2o
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/home/$USER/usr -DSPDLOG_BUILD_SHARED=ON
cmake --build build --parallel
cmake --install build
```

Python library:

```
pip install -U g2o-python
```
Refs: [1](https://github.com/miquelmassot/g2o-python)

## g2o Format

In a graph-based optimization problem, you typically have a set of variables (also known as nodes) and constraints (also known as edges) between these variables. The g2o format provides a way to express these variables and constraints in a text-based file.

The g2o file consists of several sections, each indicated by a specific keyword:


### Vertices
syntax: 

`TAG ID CURRENT_ESTIMATE`

Examples:


### 2D Robot Pose

`VERTEX_SE2 i x y theta`
`VERTEX_SE2 4 0.641008 -0.011200 -0.007444`

### 2D Landmarks / Features

`VERTEX_XY i x y`





### 3D Robot Pose

`VERTEX_SE3:QUAT i x y z qx qy qz qw`

`VERTEX_SE3:QUAT 0 0 0 0 0 0 1`

### 3D Point


`VERTEX_TRACKXYZ i x y z`

## Edges / Constraints

`TAG ID_SET MEASUREMENT INFORMATION_MATRIX`

The odometry of a robot connects subsequent vertices with a relative transformation which specifies how the robot moved according to its measurements. For a compact documentation we employ the following helper function.

`EDGE_SE2 i j x y theta info(x, y, theta)`

Where <img src="https://latex.codecogs.com/svg.latex?z_%7Bij%7D%20%3D%20%5Bx%2C%20y%2C%20%5Ctheta%5D%5ET" alt="https://latex.codecogs.com/svg.latex?z_{ij} = [x, y, \theta]^T" /> is the measurement moving from <img src="https://latex.codecogs.com/svg.latex?x_i" alt="https://latex.codecogs.com/svg.latex?x_i" />  to <img src="https://latex.codecogs.com/svg.latex?x_j" alt="https://latex.codecogs.com/svg.latex?x_j" />, i.e. <img src="https://latex.codecogs.com/svg.latex?x_j%20%3D%20x_i%20%5Coplus%20z_ij" alt="https://latex.codecogs.com/svg.latex?x_j = x_i \oplus z_ij" />



`EDGE_SE2 24 25 0.645593 0.014612 0.008602 11.156105 -3.207460 0.000000 239.760661 0.000000 2457.538661`


`EDGE_SE3:QUAT 0 1 0 0 0 0 0 0 0 1 0 0 0 0.1 0 0 0 0.1 0 0.1 0.1 0.1 0.1`

`ID_SET`: is a list of vertex IDs which specifies to which vertices the edge is connected.

`MEASUREMENT` : The information matrix or precision matrix which represent the uncertainty of the measurement error is the inverse of the covariance matrix. Hence, it is symmetric and positive semi-definite. We typically only store the upper-triangular block of the matrix in row-major order. For example, if the information matrix <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7B%5COmega%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{\Omega}"  /> is a `3x3`  


<img src="https://latex.codecogs.com/svg.latex?q_%7B11%7D%20%5C%3B%20q_%7B12%7D%20%5C%3B%20q_%7B13%7D%20%5C%3B%20q_%7B22%7D%20%5C%3B%20q_%7B23%7D%20%5C%3B%20q_%7B33%7D" alt="https://latex.codecogs.com/svg.latex?q_{11} \; q_{12} \; q_{13} \; q_{22} \; q_{23} \; q_{33}" />

[Examples](https://github.com/RainerKuemmerle/g2o/tree/pymem/python/examples)


Refs: [1](https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D)



### Examples

[Here](https://github.com/RainerKuemmerle/g2o/tree/pymem/python/examples)


### Tools

- g2o_viewer
- g2o_incremental
- slam2d_g2o 




# g2o 
g2o is a C++ framework for optimizing graph-based nonlinear error functions
```
git clone https://github.com/gabime/spdlog.git
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/home/behnam/usr -DSPDLOG_BUILD_SHARED=ON
cmake --build build
cmake --install build
```

`sudo apt install libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5`


```
git clone https://github.com/RainerKuemmerle/g2o/
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/home/behnam/usr -DSPDLOG_BUILD_SHARED=ON
cmake --build build
cmake --install build
```

## g2o-python


```
pip install -U g2o-python
```
Refs: [1](https://github.com/miquelmassot/g2o-python)


## File Format SLAM 2D

In a graph-based optimization problem, you typically have a set of variables (also known as nodes) and constraints (also known as edges) between these variables. The g2o format provides a way to express these variables and constraints in a text-based file.

The g2o file consists of several sections, each indicated by a specific keyword:


## Vertices
syntax: 

`TAG ID CURRENT_ESTIMATE`

Examples:


### 2D Robot Pose

`VERTEX_SE2 i x y theta`

`VERTEX_SE2 4 0.641008 -0.011200 -0.007444`

### 2D Landmarks / Features

`VERTEX_XY i x y`

### Edges / Constraints

`TAG ID_SET MEASUREMENT INFORMATION_MATRIX`

The odometry of a robot connects subsequent vertices with a **relative transformation** which specifies how the robot moved according to its measurements. For a compact documentation we employ the following helper function.

`EDGE_SE2 i j x y theta info(x, y, theta)`

Where <img src="https://latex.codecogs.com/svg.latex?z_%7Bij%7D%20%3D%20%5Bx%2C%20y%2C%20%5Ctheta%5D%5ET" alt="https://latex.codecogs.com/svg.latex?z_{ij} = [x, y, \theta]^T" /> is the measurement moving from <img src="https://latex.codecogs.com/svg.latex?x_i" alt="https://latex.codecogs.com/svg.latex?x_i" />  to <img src="https://latex.codecogs.com/svg.latex?x_j" alt="https://latex.codecogs.com/svg.latex?x_j" />, i.e. <img src="https://latex.codecogs.com/svg.latex?x_j%20%3D%20x_i%20%5Coplus%20z_ij" alt="https://latex.codecogs.com/svg.latex?x_j = x_i \oplus z_ij" />



`EDGE_SE2 24 25 0.645593 0.014612 0.008602 11.156105 -3.207460 0.000000 239.760661 0.000000 2457.538661`



## File Format SLAM 3D

### 3D Robot Pose

`VERTEX_SE3:QUAT i x y z qx qy qz qw`

`VERTEX_SE3:QUAT 0 0 0 0 0 0 1`

### 3D Point


`VERTEX_TRACKXYZ i x y z`

### Edges / Constraints


`EDGE_SE3:QUAT 0 1 0 0 0 0 0 0 0 1 0 0 0 0.1 0 0 0 0.1 0 0.1 0.1 0.1 0.1`

`ID_SET`: is a list of vertex IDs which specifies to which vertices the edge is connected.

`MEASUREMENT` : The information matrix or precision matrix which represent the uncertainty of the measurement error is the inverse of the covariance matrix. Hence, it is symmetric and positive semi-definite. We typically only store the upper-triangular block of the matrix in row-major order. For example, if the information matrix <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7B%5COmega%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{\Omega}"  /> is a `3x3`  


<img src="https://latex.codecogs.com/svg.latex?q_%7B11%7D%20%5C%3B%20q_%7B12%7D%20%5C%3B%20q_%7B13%7D%20%5C%3B%20q_%7B22%7D%20%5C%3B%20q_%7B23%7D%20%5C%3B%20q_%7B33%7D" alt="https://latex.codecogs.com/svg.latex?q_{11} \; q_{12} \; q_{13} \; q_{22} \; q_{23} \; q_{33}" />

[Examples](https://github.com/RainerKuemmerle/g2o/tree/pymem/python/examples)


Refs: [1](https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D)



