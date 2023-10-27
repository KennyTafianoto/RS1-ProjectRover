# Download dependicies 
```
sudo apt install libopencv-dev python3-opencv

sudo apt-get install ros-noetic-vision-opencv 

sudo apt-get install ros-noetic-cv-bridge

# Build your catkin_ws and source it
# Run turtlebot 
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Running colour detection from catkin_ws
python src/RS1-ProjectRover/rs_colourdetection/test.py

```

# How does it work
* Colour space: HSV (Hue, Saturation, Value)
### Callback
* When there's data uploaded to selected topic, it will take an image from the topic
* It calls detect_colours function
* Once the centroid of the colours are returned, it plots a circle in the middle to represent the centroid
### Detect Colours
* Set the lower and upper bounds for colours we want. Refer to resources for RGB value chart
* Converts cv__image to hsv_image for analysis
* Masks off the colours within the specified range
* Find contour to allow us to determine midpoint

# Colour Theory
* Hue: color appearance parameters
* All colours are a variant of unique hues: red, orange, yellow, green, blue, purple 
* Various colour spaces to represent an image. E.g. RGB, HSV, BGR
* Saturation is represented between a scale of 0 to 255. Pure colour has no gray mixed in it, the greater the amount of grey, the lesser the saturaiton. 
* Value: measure brightness of a colour between a scale of 0 to 255. Max brightness = white, minimum brightness = black. 


# Resources 
[Colour theory and tutorial](https://medium.com/featurepreneur/colour-filtering-and-colour-pop-effects-using-opencv-python-3ce7d4576140)
[RGB Values for deciding colour bounds](https://www.teoalida.com/database/Excel-colors-with-RGB-values-by-Teoalida.png)
[Contour](https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html)

