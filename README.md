# Animation Render

Evaluation framework for the mdltracker project.
Uses the blender python module to render textured objects.
Templates can be obtained by using the flickr API.

## Getting Started


### Prerequisites

- Ubuntu 16.04
- ROS Kinetic: http://wiki.ros.org/kinetic
- OpenCV: https://opencv.org

### Installing

In catkin workspace:
catkin build

## Running the tests

In catkin workspace:
catkin run_tests

## Commands
The node can be controlled via the /animation_render/control topic.
Commands:
- CreateBackgroundList
- CreateTemplateList
- CreateLists
- DownloadBackgrounds
- DownloadTemplates
- DownloadImages
- RenderVideos
- StartTest

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
