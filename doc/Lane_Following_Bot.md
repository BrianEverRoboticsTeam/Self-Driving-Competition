# Lane Following Bot
This documentation described the approches of our lane following behavior on turtlebot 2 (Kobuki) for the self-drving Competition.


## Overview
Our Self Driving Bot is a robot (turtlebot 2) that could self driving in a conditional lane and able to stop when the red light (traffic signal light similator) is on. Specifically on the competition, the lane will be twice wider than turtlebot 2, and it will form a loop in the match room. The inner border of loop will be a yellow dash line, and the outter border will be a white solid line. The goal of our robot is to driving inside the lane as fast as possible. 


## Resource Set Up 
We used two cameras, one from a PremeSence RGB-D sensor (Asus Xtion Pro) and the other camera is a Logitech HD Webcam C270, on our self-driving bot. Those sensor can provide us RGB image. The field of view from camera of our PremeSence RGB-D sensor is 58 degree horizontally and 45 degree vertivally according to [the product document](https://www.asus.com/3D-Sensor/Xtion_PRO_LIVE/specifications/). The field of view of Logitech HD Webcam C270 is 60 degree horizontally according to [
Logitech Support](http://support.logitech.com/en_us/article/17556).

The PremeSence RGB-D sensor be used as our main camera for lane detacting and the Logitech HD Webcam C270 be used as an assistant camera for red light detacting. We mounted our PremeSence RGB-D sensor at the rare of our turtlebbot 2 with roughly 36 cm distance to the ground and lower it's facing direction about 45 degree to the gound. By doing this, our robot can see the lane from 0 meter to about 2 meters ahead. This view distance should be able to provide enough information for our robot to react in a sharp turning angle and it could also minimize the blind area when the robot is turning. The Logitech HD Webcam C270 be mounted on the lowest disk board of turtlebot 2, at the back as well, but only has about 16 cm to the ground, facing down to the disk surface. The purpose we facing this camera down to disk surface is to avoid the interference from not self-luminous object. Only the ray from self-luminous object, such as the red light, could be strong enough to be reflacted by the disk surface and capture by our camera.

Our turtlebot 2 is distributed by Clearpath Robotics. Referred to the [Kobuki User Guide](https://www.google.ca/url?sa=t&rct=j&q=&esrc=s&source=web&cd=9&ved=0ahUKEwjvhqLH1v_RAhVOw2MKHYAFAY4QFghCMAg&url=https%3A%2F%2Fdocs.google.com%2Fdocument%2Fexport%3Fformat%3Dpdf%26id%3D15k7UBnYY_GPmKzQCjzRGCW-4dIP7zl_R_7tWPLM0zKI&usg=AFQjCNFo0O5d312q_k2JDorv5Q0cIMiZ7A&bvm=bv.146094739,d.cGc&cad=rja), the maximum linear speed of our turtlebot is 0.7 m/s, and the maximum angular speed is 180 degree/s (about 3.14 radiance/s).


## Planning & Problems
The first thing we planed to do is detacting border lines by color. We needed two filters to do that, one for the white solid line and the other for the yellow dash line. This sound reasonable and easy to implement, But the white filter will actually treat all bright area as white color. This is a trouble since our main camera is running under auto-exposure. So, there could be a frame accidentally over expose and most of area of that frame will be considered as white color by our white color filter. The yellow color filter won't guarantee 100% working, either. It could work well when the yellow line is appear near the center of the frame but not when near the edge of a frame.

We have tried turning off auto-exposure and it turns out some of the spot is too dark to get any useful information in the match room. We also tried turning off auto-white balance to see if we got a better result from yellow color filter. However, we found the frame became warmer so it's more obstruct to our yellow color filter. For these two reasons, we designed to use defualt setting of our main camera, which is auto-exposure and auto-white banlance.

Then, we planned to do some image processing in order to improve our color filters. Specifically, we planned to apply equilization on RGB channels. Doing this will separate the color as much as possible, so it's easier for human to pick a color range and improveing the performance of color filters. This method been prove working well during the competition. Even thought the filters are still not 100% working, but it will work about 90% of the time. This is a huge improvement. 

Solved the filter problem, we should able to control our robot and keep it in lane for 90% of the time based on those filtered information. However, the ratio of that in-lane time is actually lower. The reason we believe is due to the poor image quality near the edge of a frame. The border line will appear near the edge of a frame if we keep our robot at the center to the lane. But, the accurancy for our color fiter is lower at the edge of a frame. So, it seems better to keep our robot closer to a border line and use another line as an assistance. We pick the white solid line as the mainly line to follow since it's the outter border which will be visible to our robot for the most of the time. 

For the red light detaction, we plan to detact the density of red color in a frame and use a threshold to determine whether there is a red light is on. The only problem is if we facing our camera directly to the light, it will be too bright to detact enough density of red color from the light and this will easier be obstructed by other red color in the same frame. However, all the problems been resolved when we facing our camera to the disk surface.


## Approching Details

##### Image Processing
Image processing is cricial for our robot's control algorithm since the quality of color filtering is depends on it. We applied equalizaiton process on all color channels (RGB) of the original frame. 

## {pig: image_processing_before_and_after}

As you can see from the images above, his process actually higher the saturation of color in frame. Although this might not seem different by the robot, but it's easier for human to define a color range. The detail code of image processing been attached bwlow,

```python
# blur image to reduce the noise
image = cv2.GaussianBlur(image,(13,13),0)

# equalize RGB channels
image[:,:,2] = cv2.equalizeHist(image[:,:,2])
image[:,:,1] = cv2.equalizeHist(image[:,:,1])
image[:,:,0] = cv2.equalizeHist(image[:,:,0])
```
 
##### Control Method
Our robot mainly following the white line and use yellow line as assitance to find white line. It calculate the image moment on white color filter and yellow color fiter if there is enough density of the corresponding color. Then, the centre of white and yellow as well as their errors to the central axis on x-axis could be found. Finally, this error be used in our proportional control of angular speed. Detail code have been attached below,

```python
 # use moment to find error to the center
M_white = cv2.moments(mask_white)
M_yellow = cv2.moments(mask_yellow)

if M_white['m00'] > 10000:
    err_white = find_central(M_white,image, w, (0,255,255)) - 180
else:
    err_white = 0

if M_yellow['m00'] > 200000:
    err_yellow = find_central(M_yellow,image, w, (255,0,0)) + 400
else:
    err_yellow = 0

# send command to robot based on the err
err = err_white
if(err_yellow!=0):
    err = err_yellow

twist.linear.x = 0.6
twist.angular.z = -float(err) / 190
```


## Result and Discussion




