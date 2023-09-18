
- SLAM Loop Closure
In Simultaneous Localization and Mapping (SLAM), a closure is an observation that is made about the environment that closes a loop in the estimated trajectory of the robot. When a closure is detected, the system can correct for accumulated drift in the estimated position of the robot and improve the accuracy of the map.

Here is an example of how you could implement loop closure detection in SLAM using Python:

```
import numpy as np

class LoopCloser:
    def __init__(self, max_distance=0.1, max_angle=np.pi / 6):
        self.max_distance = max_distance
        self.max_angle = max_angle
        self.keyframes = []
        self.loop_closure = None

    def process_frame(self, frame):
        # Calculate the relative pose between the current frame and the last keyframe
        relative_pose = calculate_relative_pose(frame, self.keyframes[-1])
        distance, angle = calculate_distance_and_angle(relative_pose)

        # If the distance and angle are within the threshold, consider it a loop closure
        if distance < self.max_distance and angle < self.max_angle:
            self.loop_closure = (len(self.keyframes) - 1, frame)
            return True
        else:
            self.keyframes.append(frame)
            return False

    def get_loop_closure(self):
        return self.loop_closure

def calculate_relative_pose(frame1, frame2):
    # Calculate the relative pose between two frames using motion models,
    # feature matching, or other methods
    relative_pose = ...
    return relative_pose

def calculate_distance_and_angle(relative_pose):
    # Calculate the distance and angle between two poses
    distance = ...
    angle = ...
    return distance, angle
```
In this example, LoopCloser is a class that implements the loop closure detection algorithm. The class has a member process_frame which takes a new frame as input and checks if it is a loop closure. If the distance and angle between the current frame and the last keyframe are within the threshold, it is considered a loop closure. The class also has a member get_loop_closure which returns the current loop closure, if any. The loop closure is stored as a tuple of the index of the keyframe and the current frame.

The function calculate_relative_pose takes two frames as input and calculates the relative pose between them. The function calculate_distance_and_angle takes a relative pose as input and calculates the distance and angle between the two poses.

This is a simple example of how you could implement loop closure detection in SLAM. In practice, there are many variations and optimizations that can be made to the algorithm, such as using more sophisticated methods for detecting loop closures, using a bag-of-words representation for features, and using pose graph optimization to correct for drift in the estimated trajectory.



Refs: [1](http://www.cds.caltech.edu/~murray/courses/me132-wi11/me132a_lec16.pdf)


  
- Matrix Lie Groups for Robotics
Refs: [1](https://www.youtube.com/watch?v=NHXAnvv4mM8&list=PLdMorpQLjeXmbFaVku4JdjmQByHHqTd1F&index=8)   




# GraphSLAM

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

<img src="images/towards_the_information_form.jpg" alt="images/towards_the_information_form.jpg" />

<br/>



