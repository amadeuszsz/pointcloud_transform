#pointcloud_transform

Pointcloud rotation using quaternions.

# Usage:
1. Run node
   rosrun pointcloud_transform pointcloud_transform
2. Publish transformation as quaternion
   rostopic pub -r 100 /angle geometry_msgs/Quaternion "x: 0.0 y: 0.707 z: 0.0 w: 0.707"
3. Check new rotated topic
   /laser_cloud_surround_rotated
   
