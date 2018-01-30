# fishtarget_ros
This repository contains the ROS package (and Arduino code) for running the automated fish target system at Villanova University.

# Using the package

To launch the fish target system, enter the following command into a terminal:

```$ roslaunch fish_target fish_target.launch```

You will see a window pop up that allows you to monitor the webcam view and the filtered sensor value.

To record data from the system, use the "record" button in the lower left hand corner. You will want to save the "bag" file somewhere that you'll find it later, and make sure to record the topics "/fishtarget/targetinfo" and "/fishtarget/overlay_image" to your file.

To end a session, pull up the terminal window and hit Control+C.

To "play back" data, enter the following command in a terminal:

```$ roslaunch fish_target fish_replay.launch```

Then, use the widget in the lower-left-hand corner of the window that pops up to load your file. Before you hit "play" to review your data, right click on the bag file window and select "publish all."
