# Representing Robot Pose
<img src="images/representing_robot_pose1.png" width="50%" height="50%" />



# Quaternions Inverse Pose

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

# Quaternions Relative Pose

If Pose <img src="https://latex.codecogs.com/svg.latex?C" alt="C" />  express in Frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />  and pose of <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> expressed in <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />  using quaternions,  equation for finding the pose <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> expressed in <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />  using quaternions





1. **Rotations**:
Let's define the following quaternions for the rotations:
- <img src="https://latex.codecogs.com/svg.latex?Q%5E%7BA%7D_%7BB%7D" alt="Q^{A}_{B}" /> is the quaternion representing the rotation of frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> with respect to frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" /> (<img src="https://latex.codecogs.com/svg.latex?B" alt="B" />'s rotation expressed in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />).
- <img src="https://latex.codecogs.com/svg.latex?Q%5E%7BB%7D_%7BC%7D" alt="Q^{B}_{C}" /> is the quaternion representing the rotation of frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> with respect to frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> (<img src="https://latex.codecogs.com/svg.latex?C" alt="C" />'s rotation expressed in frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />).

The combined rotation of frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> with respect to frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />, <img src="https://latex.codecogs.com/svg.latex?Q%5E%7BA%7D_%7BC%7D" alt="Q^{A}_{C}" /> , is given by:
<img src="https://latex.codecogs.com/svg.latex?Q%5E%7BA%7D_%7BC%7D%20%3D%20Q%5E%7BA%7D_%7BB%7D%20%5Cotimes%20Q%5E%7BB%7D_%7BC%7D" alt="Q^{A}_{C} = Q^{A}_{B} \otimes Q^{B}_{C} " />



2. **Translations (positions)**:
If you have the positions:
- <img src="https://latex.codecogs.com/svg.latex?P%5E%7BA%7D_%7BB%7D" alt="P^{A}_{B}" /> is the position of point <img src="https://latex.codecogs.com/svg.latex?B" alt="B" /> (or frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />'s origin) expressed in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />.
- <img src="https://latex.codecogs.com/svg.latex?P%5E%7BB%7D_%7BC%7D" alt="P^{B}_{C}" /> is the position of point <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> (or frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" />'s origin) expressed in frame <img src="https://latex.codecogs.com/svg.latex?B" alt="B" />.

The position of point <img src="https://latex.codecogs.com/svg.latex?C" alt="C" /> (or frame <img src="https://latex.codecogs.com/svg.latex?C" alt="C" />'s origin) expressed in frame <img src="https://latex.codecogs.com/svg.latex?A" alt="A" />, <img src="https://latex.codecogs.com/svg.latex?P%5E%7BA%7D_%7BC%7D" alt="P^{A}_{C}" />, when considering rotations, is:
<img src="https://latex.codecogs.com/svg.latex?P%5E%7BA%7D_%7BC%7D%20%3D%20P%5E%7BA%7D_%7BB%7D%20&plus;%20Q%5E%7BA%7D_%7BB%7D%20%5Cotimes%20P%5E%7BB%7D_%7BC%7D%20%5Cotimes%20%28Q%5E%7BA%7D_%7BB%7D%29%5E%7B-1%7D" alt=" P^{A}_{C} = P^{A}_{B} + Q^{A}_{B} \otimes P^{B}_{C} \otimes (Q^{A}_{B})^{-1} " />


Where <img src="https://latex.codecogs.com/svg.latex?%28Q%5E%7BA%7D_%7BB%7D%29%5E%7B-1%7D" alt="(Q^{A}_{B})^{-1}" /> denotes the conjugate (or inverse) of the quaternion <img src="https://latex.codecogs.com/svg.latex?%5C%28%20Q%5E%7BA%7D_%7BB%7D%20%5C%29" alt="\( Q^{A}_{B} \)" />.



