# ROS packages for detection with apriltag

## Start camera

### Xtion

Run: 

```
roslaunch openni2_launch openni2.launch camera:=asus_xtion
```


### Realsense

Not supported yet.

## Camera registration with the robot

## Create a tag

In order to generate and print a pattern you can set the parameters in `apriltag_detector/config/params.yml` under `make_pattern`. Then run:

```
roslaunch apriltag_launch make_pattern.launch
```

## Run detection
