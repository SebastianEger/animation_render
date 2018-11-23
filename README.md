# Animation Render

Evaluation framework for the mdltracker project.
Uses the blender python module to render textured objects.
Templates can be obtained by using the flickr API.

## Getting Started


### Prerequisites

- Ubuntu 16.04
- ROS Kinetic: http://wiki.ros.org/kinetic
- Python 3.5 (comes with Ubuntu)
- OpenCV: https://opencv.org

### Installing

In catkin workspace:
```
catkin build
```
All dependencies are build automatically.
The build process can take awhile.
The blender python module is installed at **/usr/include/python3.5/site-packages**.

## Commands
The node can be controlled via the **/animation_render/control** topic, e.g:
```
rostopic pub /animation_render/control std_msgs/String CreateBackgroundList
```
Commands:
```
CreateBackgroundList
```
Creates list of image URLs from FLickr with background keywords.
```
CreateTemplateList
```
Creates list of image URLs from FLickr with template keywords.
```
CreateLists
```
Create background and template list.
```
DownloadBackgrounds
```
Download all images stored as URL in the background list.
```
DownloadTemplates
```
Download all images stored as URL in the template list.
```
DownloadImages
```
Download all background and template images.
```
RenderVideos
```
Renders all videos with given template and background images.
```
StartTest
```
Start test series. Videos have to be rendered first.
```
ExportGroundTruth
```
Exports ground truth data to ~/ground_truth
```
EvaluateTemplates
```
Evaluate all templates for their gradient

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
