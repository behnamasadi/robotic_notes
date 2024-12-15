import rerun as rr
import rerun.blueprint as rrb
import numpy as np
import time


# Initialize Rerun session
rr.init("rerun_example_different_data_per_time", spawn=True)
rr.reset_time()  # Clears all set timeline info.

for t in range(10):
    # Generate a random point in 3D space
    point = np.random.rand(1, 3)  # Random position in [0, 1] for each axis
    
    # Set the timeline step
    rr.set_time_sequence("time", t)
    time.sleep(1)  # Wait for 1 seconds
   
    # Log the point with a radius of 0.1
    rr.log(f"point{t}", rr.Points3D(point, radii=0.1))


