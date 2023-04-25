# Lab 1: Custom Messages


## Purpose
This lab will provide practice in creating custom messages.  You will use provided code that listens for events from the mouse.  Specifically, the code is listening to the position of the cursor, and any buttons pressed.  You will create a custom message that is going to pass the cursor position and mouse button events across the /mouse_info topic.  You will develop a controller that will subscribe to the /mouse_info topic and then publish a Twist message to /cmd_vel according to the requirements in the lab handout.

#### Additional Requirements
The mouse controller will only be activated by scrolling down with the mouse.

The mouse controller will be immediately deactivated by scrolling up with the mouse.

If the mouse controller is deactivated, the robot will stop all movement, until an override by another module.

You don't need to do handle the override at this point, but your controller code should be flexible enough that it can handle additional capability in the future.

## Report
Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.

## Turn-in Requirements
**[25 points]** Demonstration of mouse control of Turtlebot (preferably in person, but can be recorded and posted to Teams under the Lab1 channel).

**[50 points]** Report via Gradescope.

**[25 points]** Code: push your code to your repository. Also, include a screen shot or the raw text of the **controller.py** and **mouse_controller.py** files at the end of your report.
