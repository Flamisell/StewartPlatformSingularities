# Stewart Platform Singularities Finder
This notebook is propedeutic for the design of a stewart platform's controller with local singularity avoidance.

When the local condition index of the transpose Jacobian matrix approaches zero, the required actuator forces become infinitely large, making it impossible for the platform to move. In a real-world scenario, this results in the platform becoming stuck in specific configurations, often requiring manual intervention to release it. Avoiding this points is crucial for a continuos motion.

Singularities are determined by the Jacobian matrix of the platform, which in turn depends on the platform's geometry and configuration. Finding these configurations in advance allows us to create an offline set of configurations to avoid.

This notebook offers a set of tools and techniques to identify poses that lie on singularity planes. Once the singular poses are identified, it becomes possible to find the nearest singular configuration for a given platform pose and visualize the singularity planes within the workspace, assuming a fixed orientation or position.

**Closest Singularity**

<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/stewart1.png" width="400"> <img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/Stewart22.png" width="450">

**Singularities Plane**

<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/PlotPos.png" width="400"> <img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/PlotOr.png" width="400">

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
  - [Singularity Search](#singularity-search)
    - [Platform Initialization](#platform-initialization)
    - [Find Singularities](find-singularities)
    - [Filter Singularities](filter-singularities)
  - [Closest Singularity Search and Visualization](#closest-singularity-search-and-visualization)
- [Class Methods Overview](#class-methods-overview)

## Installation

Ensure you have the required libraries:

```bash
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from google.colab import drive
from scipy.spatial.distance import cdist

```

## Usage

### Singularity Search
The steps to obtain an offline set of singular configurations is presented here below.

#### Platform Initialization
This class has multiple purposes which will not be discussed here but can be found in my Stewart Platform Class repository, the key function that is used in this notebook is the *getSingularityWorkspace* function.
By defining the workspace boundaries (in terms of position and orientation), the local condition index of the transposed Jacobian matrix is calculated for each pose telling us when Jabobian becomes singular.

To create an instance of the StewartPlatform class, you need to provide the following parameters:
- r_b: Radius of the base.
- phi_b: Angle between base joints.
- r_p: Radius of the platform.
- phi_p: Angle between platform joints.

It is also important to define limits on workspace position and orientation, this limits will define the boundaries of the search.
```
# Define parameters
r_b = 0.5  # Radius of base
phi_b = 50  # Angle between base joints
r_p = 0.3  # Radius of platform
phi_p = 80  # Angle between platform joints

# Create Stewart Platform instance
platform = StewartPlatform(r_b, phi_b, r_p, phi_p) # Initialize platform
pose=np.array([0,0,0.5,10,0,0]) # Define pose
lengths= platform.getIK(pose) # Find Joint position
k=platform.getLocalConditionIndexT() # Tells you how close you are to a singularity

# Define workspace and orientation limints
workspace_limits = [-0.5, 0.5, -0.5, 0.5, 0.1, 0.6]
orientation_limits = [-10, 10, -10, 10, -10, 10]
x_min, x_max, y_min, y_max, z_min, z_max = workspace_limits
roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max = orientation_limits
```
Mount the drive
```
from google.colab import drive
drive.mount('/content/drive')
```
#### Find Singularities
Use *getSingularityWorkspace* to search for any pose which has a low local condition index. Note: this can be computationally expensive.
```
N_pos=10 # Discretization for workspace coordinates
N_orient=10 # Discretization for orientation coordinates
Holder=platform.getSingularityWorkspace(workspace_limits,orientation_limits,N_pos,N_orient) # find singularities in all space
```
Save the singularities onto drive for safety.
```
 with open('/content/drive/My Drive/Github/singularities_2.txt', 'w') as f:
     np.savetxt(f, Holder)
```
Load the whole singularity file.
```
with open('/content/drive/My Drive/Github/singularities_2.txt', 'r') as f:
    singularities = np.loadtxt(f)
```
#### Filter Singularities
*First Filtering*

Slicing the matrix, removing the borders of the workspace (external singularities) and selecting only the poses with low local condition index.
```
keep_mask = (singularities[:, 6] < 0.001) & (singularities[:, 2] > z_min)& (singularities[:, 2] < z_max) & (singularities[:, 0] < x_max)& (singularities[:, 0] > x_min)& (singularities[:, 1] < y_max) & (singularities[:, 1] > y_min)
interest_points_with_index=singularities[keep_mask]
```
*Second filtering*

We normalize the singularities between the borders of the workspace, we then calculate the distance matrix between all of them and remove the configurations which are too close to each other.

```
from scipy.spatial.distance import cdist

# take only poses
interest_points=interest_points_with_index[:,:6]
#normalize the singularities poses
mins=np.array([x_min,y_min,z_min,roll_min,pitch_min,yaw_min])
maxs=np.array([x_max,y_max,z_max,roll_max,pitch_max,yaw_max])
normalized_interest_points = (interest_points - mins) / (maxs - mins)

threshold=0.3
distances = cdist(normalized_interest_points, normalized_interest_points)
# To avoid self-comparison, set the diagonal to a large value
np.fill_diagonal(distances, np.inf)

index_holder=[]
interest_points_holder=np.copy(interest_points_with_index)
keep_mask = np.ones(normalized_interest_points.shape[0], dtype=bool)
for i in range(normalized_interest_points.shape[0]):
    interest_points_holder[i][6]=i
    if keep_mask[i]:
        # Find indices of vectors within the threshold distance
        close_indices = np.where(distances[i] < threshold)[0]
        # Set keep_mask to False for these indices
        keep_mask[close_indices] = False
        # Ensure we don't set the current vector's mask to False
        keep_mask[i] = True


interest_points_filtered=interest_points_holder[keep_mask] # kept singularities
interest_points_deleted=interest_points_holder[~keep_mask] # removed singularities
```
Save singularities in task space and in joint space.
```
# Save the filtered singularities as task singularities
singularities_task_space=np.copy(interest_points_filtered[:,:6])
with open('/content/drive/My Drive/Github/filtered_singularities_task_space_2.txt', 'w') as f:
    np.savetxt(f, singularities_task_space)

# Save the filtered singularities as joint singularities
singularities_joint_space=np.copy(interest_points_filtered[:,:6])
for i in range(len(singularities_task_space)):
  singularities_joint_space[i,:]=np.linalg.norm(platform.getIK(singularities_task_space[i,:]),axis=1)
with open('/content/drive/My Drive/Github/filtered_singularities_joint_space_2.txt', 'w') as f:
    np.savetxt(f, singularities_joint_space)
```
### Closest Singularity Search and Visualization
In this section is shown how to search for the closest singularity given a configuration of the platform and how to visualize the singularity space in both position and orientation space. Note: stewart platform's singularities live in R6, to visualize them we need to fix either position or orientation of the platform.

Load singularities from the drive
```
with open('/content/drive/My Drive/Github/filtered_singularities_task_space_2.txt', 'r') as f:
    singularities_task_space = np.loadtxt(f)

with open('/content/drive/My Drive/Github/filtered_singularities_joint_space_2.txt', 'r') as f:
    singularities_joint_space = np.loadtxt(f)
```
Find the closest singularity to defined position.

Check local condition index and the actuator forces in pose and singularity.
```
# find the closest singularity.
pose=np.array([0,0.35,0.2,0,0,0])
mins=np.array([x_min,y_min,z_min,roll_min,pitch_min,yaw_min])
maxs=np.array([x_max,y_max,z_max,roll_max,pitch_max,yaw_max])
mins = np.array(mins)
maxs = np.array(maxs)

# Normalize each component
normalized_pose = (pose - mins) / (maxs - mins)
# print(normalized_pose)
normalized_singularities=(singularities_task_space-mins)/(maxs-mins)
# print(normalized_singularities)
distances = np.linalg.norm(normalized_singularities - normalized_pose, axis=1)
# print(distances)
# # Find the index of the minimum distance
min_index = np.argmin(distances)
# print(distances[min_index])

# # Return the closest vector
closest_vector = singularities_task_space[min_index]


k=platform.getLocalConditionIndexT()
lengths=platform.getIK(pose)
k=platform.getLocalConditionIndexT()
Fg=np.array([-10,0,0,0,0,0])

joint_forces=platform.getActuatorForces(Fg)
platform.plot()
print("pose :", pose)
print("local condition number T :", k)
print("joint_forces :", joint_forces)


lengths=platform.getIK(closest_vector)
platform.plot()
k=platform.getLocalConditionIndexT()
joint_forces=platform.getActuatorForces(Fg)
print("closest singularity :", closest_vector)
print("local condition number T :", k)
print("joint_forces :", joint_forces)
```
<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/stewart1.png" width="400"> <img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/Stewart22.png" width="450">


It is possible to plot 3d the singularity planes as long as we fix either position or orientation.
We can use plotly libraries to clearly see the planes. I will leave the code directly in the colab file.
```
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
```
**Fixing Position**
```
# Choose a position
position = np.array([-0.45,0.1,0.2])

N=10 # discretization of space
choice=6 # choose choice 6 for local condition index
orientation_limits = [-10, 10, -10, 10, -10, 10] # use same orientation limits as singularity search
roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max = orientation_limits

# use the getIndexWorkspaceOrientation function to see how the local condition index changes when orienting the platform
workspace_indices_orientation = platform.getIndexWorkspaceOrientation(position, orientation_limits, N, choice)
```
<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/FixingPos.png" width="400">

**Fixing Orientation**
```
# Choose an orientation
orientation = np.array([8,7,5]) # RPY
N=10 # discretization of space
choice=6 # choose choice 6 for local condition index
workspace_limits = [-0.5, 0.5, -0.5, 0.5, 0.1, 0.6] # use same workspace limits as singularity search
x_min, x_max, y_min, y_max, z_min, z_max = workspace_limits

# use the getIndexWorkspacePosition function to see how the local condition index changes when positioning the platform
workspace_indices_position = platform.getIndexWorkspacePosition(orientation, workspace_limits, N, choice)
```
<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/FixingOr.png" width="400">

## Class Methods Overview
- **getIK(pose):** Computes inverse kinematics.
- **getFK(starting_pose, lengths_desired):** Computes forward kinematics.
- **getLocalConditionIndexT():** Calculates the local condition index of Transposed Jacobian
- **getPlatformForces(F_actuators):** Computes platform forces from actuator forces.
- **getSingularityWorkspace(workspace_limits,orientation_limits,N_pos,N_orient):** Evaluate singularities over a range of positions in the workspace.
- **plot():** Plots the Stewart platform configuration.
