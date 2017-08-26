## Project: Search and Sample Return
---


**The goals / steps of this project are the following:**

**Training / Calibration**

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.


## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

Here we go :)

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
I have written a method `color_range_hsv` (cell 8) utilising openCV range selection after converting the image to HSV colorspace for more accurate color selection.

For obstacle detection, i considered all pixels not sand as obstacles. Therefore a simple inversion of the navigable terrain shows all obstacles.


#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.
* Testvideo in `/output/test_mapping.mp4`

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

##### Perception step:
The methods are similar to those created in the notebook except of
- line 127: I see better results by constraining the forward vision of the rover to just a couple of meters.
- line 166 - 174: Here i populate a couple of variables used by decision for navigation and sample recovery.

##### Decision step:
I tried to implement the finite state machine pattern to simplify development.
It is a variation since i did not find a nice way to provide the agent (RoverState instance from drive_rover.py) to the State Machine on initialization. My customization involves sending the RoverState on every update function call.

- State.run updates the Rover and State properties
- State.next returns itself or another state depending on Rover's current situation

States:

`Rover_idle`:

* Idles until telemetry data is recorded.
* Leads always to `Rover_crawl_left_wall`.

`Rover_crawl_left_wall`:

* Tries to follow the left wall by adding a constant bias to the mean navigable angle.
* Increases turning angle by reducing the top speed with increasing steering angles.

Leads to:

* `Rover_return_home` if more than 80% is mapped and within 10 meters of the starting position recorded at the beginning of the program.
* `Rover_setback` if stuck for 60 cycles.
* `Rover_idle` if no telemetry is sent anymore (untested).
* `Rover_stop` if obstacle is in front.
* `Rover_approach_sample` if rock sample is close and kind of leftish.


`Rover_stop`

* Steps on brake.

Leads to:

* `Rover_approach_sample` if rock sample is close and kind of leftish.
* `Rover_crawl_left_wall` if path is clear.
* `Rover_setback` if stuck for 90 cycles.
* `Rover_steer_right` if path is not clear.

`Rover_steer_right`

* Incrementally adds right steering and reduces throttle.

Leads to:

* `Rover_approach_sample` if rock sample is close and kind of leftish.
* `Rover_crawl_left_wall` if path is clear.
* `Rover_setback` if stuck for 120 cycles.


`Rover_setback`

* Drive backwards for a short while.

Leads to:

* `Rover_crawl_left_wall` if path is clear.
* `Rover_quicksand` if position doesn't change for 25 cycles.


`Rover_quicksand`

* Spin for 150 cycles.
* Leads always to `Rover_crawl_left_wall`.


`Rover_approach_sample`

* Curved approach to avoid stucking if rock is in left turn.

Leads to:

* `Rover_collect_sample` if rock sample is close enough to collect.
* `Rover_setback` if stuck for 60 cycles.
* `Rover_crawl_left_wall` if rock is out of vision.

`Rover_collect_sample`

* Brakes and collects.
* Leads always to `Rover_crawl_left_wall`.






#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

FPS: ~36
Resolution* 800 x 600
Graphics Quality: Good

Results:

* The `Rover_done_state` is returned after at least 80% of the map is covered and the Rover to vincinity of 0.5 meters of the location he started from.
* The Rover changes into the `Rover_done_state` within 12 minutes in average, depending on the locations of the samples.
* He usually reached a top speed of around 2.6m/s.
* He usually collects at least 5 samples on the way

Points of improvement:

- The rover crashes into walls at the end of narrow corridors or sharp right turns.
Velocity and braking could further be improved maybe by introducing PID dependent on the field of view in front.

- Some edgecases in sample locations may lead to the rover endlessly trying the same approach to retrieve the rock not succeeding.

- There is one location on which my implementation is not able collect rock samples since the rover will always try to approach the sample from the left. Changing this bevaiour would break the routines for all rocks that are hidden in small left turns.
The optimisation might involve the perception to gain more understanding of the map and path planning.
![image1][stuck]
[stuck]: ./output/rover_stuck.png

- Returning home is not properly tested and is buggy.

