# tf_adder

Package to add missing transformations required by map making and localization process


## Usage

```python
roslaunch position_filter gnss.launch 

roslaunch zio_maps zio_maps.launch

rosrun tf_adder tf_adder -b 'rosbag.bag'
```

## Version
Beta 
