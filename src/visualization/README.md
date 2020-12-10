#### Fell free to improve the documentation on this project! [Need Help? Tips on how to modify README.md files here!](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)

# WHY MY MARKER ARE INVISIBLE?:
#### source: https://answers.ros.org/question/228238/reasons-for-marker-or-markerarray-to-be-invisible-in-rviz/


*  Set the scale of the marker. By default, the scale is (0,0,0), which will make the marker infinitesimal and therefore invisible.
*  Make sure the transparency is set. By default, the 'a' field of the marker color is 0, which will make the marker completely transparent.
*  Set the orientation of the marker. By default, all quaternion parameters are zero, which is an invalid quaternion. Set w to 1 to initialize the orientation.
*  Update the time stamp of the marker when you update its properties.
*  Make sure the frame ID of the marker exists in the TF tree, or is the same frame as the fixed frame in Rviz.
*  If publishing a marker array, make sure the 'id' field of each array element is unique. Otherwise, only the marker that was added last will appear.


# SOURCES AND OTHER INFORMATION:
| Website        | Links           | Sujects converd  |
| :------------- |:-------------| :-------------|
| ROS Tutorials      | http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes | Add objects to RVIZ |
| Github      | https://gist.github.com/alexsleat/1372845 | Iterate through ROS msg arrays |
| ROS Answers      |  https://answers.ros.org/question/228238/reasons-for-marker-or-markerarray-to-be-invisible-in-rviz/ | Why me markers are invisible in RVIZ |
|Github      |  https://github.com/PickNikRobotics/rviz_visual_tools | Delete markers from RVIZ |
|Github      |  https://github.com/ros-visualization/rviz/issues/865 | Delete markers from RVIZ |
| ROS Documentation | http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines | How to add lines |

