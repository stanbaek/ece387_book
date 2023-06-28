# Module 2: Linux for Robotics

### A note on this document
This document is known as a Jupyter notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

## Purpose
This Jupyter Notebook accompanies the introduction to Linux notetaker used in class. We will apply the knowledge you learned by interacting with the Ubuntu Operating System (OS) on the Master.

## Linux Commands
During class we went over a number of basic Linux commands. Open a terminal on the Master and let's practice using those commands (use the shortcut `ctrl+alt+t` to open a new terminal window or select an open terminal and hit `ctrl+shift+t` to open a new terminal tab).

When observing the terminal (or Shell) you will note the syntax `username@hostname:`(i.e., on the master: `dfec@masterX:`, on the robot: `pi@robotX`), the current working directory (i.e., `~`, which represents the home folder of the user), and lastly the '$' character and a blinking cursor. This line is the prompt and the blinking cursor indicates the terminal is active and ready for commands.

Run the commands to move to the home folder and make a new directory:

`cd` 

`mkdir my_folder`

> 💡️ **Tip:** In Linux, if you highlight a command you can paste it into the terminal by clicking the scroll wheel.

Change directories into your new folder:

`cd my_folder`

Create a new bash script we can use to drive the TurtleBot3:

`touch move_turtlebot.sh`

A bash script is a regular text file that allows you to run any command you would run within the terminal. We will use it to run a few ROS command line tools to move our TurtleBot.

We can use the Nano text editor to edit a file within the terminal. There are a number of text editors within the terminal and WWIII might be fought over which is best; some other options include Vim and Emacs. Nano is one of the more simple editors for quick edits. Feel free to use whichever works best for you, but all guidance within this course will be based on Nano.

Edit the new bash script:

`nano move_turtlebot.sh`

Copy the following into the script:

```bash
#!/bin/bash

ARG1=$1

if [ $ARG1 == 'forward' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.15
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: 0.0"
        
elif [ $ARG1 == 'rotate' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: 0.5"
        
elif [ $ARG1 == 'stop' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: 0.0"
else
echo "Please enter one of the following:
    forward
    rotate
    stop"
fi
```

Typing `ctrl+s` saves the file and then typing `ctrl+x` exits Nano.

Again, a bash script just runs commands exactly as you would within a terminal. The above code takes an argument and publishes a *Twist* message over the **/cmd_vel** topic to drive the robot accordingly.

Before running this script, let's get our ROS environment setup:

1. Open a new terminal and type `roscore`.
2. Open a new terminal tab and run our TurtleBot3 simulation: 

    `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

Now, in a new terminal, run the script you created:

`./move_turtlebot.sh`

Did you get an error? That is because the permissions have not been properly set and you do not have execute permissions. You can observe the permissions of a file by typing `ls -la`.

For the `move_turtlebot.sh` file you should see `-rw-rw-r--`. The first position indicates file type ('d' for directory, '-' for regular file), the next three positions indicate read (r), write (w), and execute (x) permissions for the file owner (u), the next three indicate permisions for the group owner of the file (g), and the last three characters indicate permissions for all other users (o).

You can change the permissions of a file or directory using the command `chmod`:

> ⌨️ **Syntax:**  `chmod <groups to assign the permissions><permissions to assign/remove> <file/folder>`

For example, if we wanted to give the Owner execute permissions, you can enter the command:

`chmod u+x move_turtlebot.sh` 

Typically we will give all users executable permissions (`chmod +x move_turtlebot.sh`). This isn't the most secure thing to do, but in our controlled environment, it isn't an issue. If you type `ls -la` now you should see the 'x' permission added for each permission group (`-rwxrwxr-x`).

Try running your script again:

`./move_turtlebot.sh rotate`

> 📝️ **Note:** You can remove permissions by utilizing the '-' character instead of '+'.

Move to your github repo that you previously created:

`cd ~/master_ws/src/ece387_master_sp23-USERNAME/master/`

Now we are going to create a new ROS package.  You learned this in the previous homework assignment, but we will continue practicing:

`catkin_create_pkg module02 std_msgs rospy roscpp`

Before we go any further, it is always a good idea to compile your workspace with the new package.  To do that, we will use a command `catkin_make` from within the top-level of the workspace:

`cd ~/master_ws`

`catkin_make`

Now go back to the `my_folder` that you created at the beginning of the lesson and create a new bash script, `bash_script.sh`, to accomplish the following:

1. Moves into the package you just created (you will need to figure out the complete path to this folder)
1. Creates a directory called `my_scripts`
1. Moves into that directory
1. Creates a file called **move_turtlebot_square.py**
1. Lists all files showing the permissions
1. Modifies the permissions of the **move_turtlebot_square.py** file so all groups have executable permissions
1. Lists all files again showing the updated permissions

Now run the script. If successful you should see the file listed without and then with execute permissions.

We are done with this script so let's remove it (you will need to either use the complete path to the file you want to remove, or be in the directory of the file you want to remove). The `rm` command can remove folders or files. If you want to learn more about a command there are two helpful tools: **Help**: `rm --help`; **Manual**: `man rm`.

Type the following to remove our bash script: 

`rm bash_script.sh`. 

> 📝️ **Note:** To delete a whole folder add the `-r` tag to remove directories and thier contents recursively (e.g., `rm -r my_folder`, but don't remove your folder just yet).

We can copy (`cp`, just like ctrl+c in a GUI) and move (`mv`, just like ctrl+x in a GUI) files and folders as well. Let's copy the `move_turtlebot.sh` to the `my_scripts` folder you created earlier:

`cp move_turtlebot.sh ~/master_ws/src/ece387_master_sp23-USERNAME/master/module02/my_scripts`

> ⌨️ **Syntax:**  `cp <source> <destination>`

> 📝️ **Note:** For the above to work, you must already be in the same folder as the `move_turtlebot.sh` file. Otherwise you have to use the absolute file path, such as `~/my_folder/move_turtlebot.sh`.

You can now delete your `my_folder` folder.

`cd ..`

`rm -r my_folder`

Change directories to your `my_scripts` folder. We can use a ROS tool, `roscd`, to change directories to ROS packages without using the absolute file path:

`roscd module02/my_scripts`

> ⌨️ **Syntax:**  `roscd <package/folder>`

Edit the **move_turtlebot_square.py** file and paste the following contents:

```python
#!/usr/bin/env python3
import rospy, time, math
from geometry_msgs.msg import Twist

class MoveTurtleBot():
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)    # 10 Hz
        
    def publish_cmd_vel_once(self):
        """
        In case publishing fails on first attempt
        """
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
                
    def shutdownhook(self):
        rospy.loginfo("Shutting down. Stopping TurtleBot!")
        self.stop_turtlebot()
        self.ctrl_c = True
        
    def stop_turtlebot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_cmd_vel_once()
        
    def move_time(self, moving_time = 10.0, lin_spd = 0.2, ang_spd = 0.2):
        self.cmd.linear.x = lin_spd
        self.cmd.angular.z = ang_spd
        
        self.publish_cmd_vel_once()
        time.sleep(moving_time)
        
    def move_square(self):
        i = 0
        
        while not self.ctrl_c:
            # Move Forward
            self.move_time(moving_time = 2.0, lin_spd = 0.2, ang_spd = 0)
            # Turn
            ang_spd = 0.5    # rad/sec
            moving_time = math.radians(90)/ang_spd
            self.move_time(moving_time = moving_time, lin_spd = 0.0, ang_spd = ang_spd)
            
        
if __name__ == '__main__':
    rospy.init_node('move_turtlebot')
    move_object = MoveTurtleBot()
    try:
        move_object.move_square()
    except rospy.ROSInterruptException:
        pass
```

There is a lot going on in this script, but hopefully after the previous lesson some of it makes sense. To summarize, the script creates a node, `move_turtlebot`, that publishes **Twist** messages to the **/cmd_vel** topic to drive the robot in a square: drive forward, turn 90 degrees, drive forward, repeat. This script will run until killed using `ctrl+c`.

The script is already executable, so you can run it using ROS!

`rosrun module02 move_turtlebot_square.py`

> 📝️ **Note:** It won't be a perfect square as the robot doesn't turn perfectly, but it will be close!

The robot is now driving in a square until the script is killed. If you select the terminal and hit `ctrl+c`, it will kill the script.

Run the script again and this time hit `ctrl+z`. You can see that the robot is still running, but the commands are not updating. This is because `ctrl+z` suspends the current process, but does not kill it. We can observe all running processes on Linux by typing `ps -faux`. As you can see, there are a lot! The `grep` command allows us to filter these processes. Try the following:

`ps -faux | grep move_turtlebot_square.py`

The first entry should be our process and the leftmost number is the process ID (PID). We can kill any process using this number and the `kill` command:

`kill PID` replacing PID with the number listed. 

If you hit enter again, you should see that the process was killed. Unfortunately, the TurtleBot will just continue to execute the last command sent, so you need to kill the simulation as well. Just select that terminal and hit `ctrl+c`.

The `grep` tool is very powerful and can be used with any Linux command. For example, if you wanted to see all turtlebot packages available to us, we could type the following:

`rospack list`

There are a lot, so this isn't very helpful, but we can filter this command!

`rospack list | grep turtlebot`

The vertical line, '|', pipes the results of the first command into the second command, so we can filter all packages looking only for turtlebot packages.

## Working with a remote machine
Later in this course you will drive your TurtleBot around unplugged from a monitor and keyboard, but you will still need access to the Raspberry Pi on the robot to run ROS nodes. One of the easiest ways to remotely access a Linux machine is through a secure shell. To create a secure shell, you need the uesrname and hostname of the computer you want to remote into. For the Raspberry Pis, all usernames are `pi` and the hostname is your robot number (e.g., `robot0`).

1. Open a terminal on your master
1. Check connectivity to the robot:

    `ping robotX`
    <br>
1. Create a secure shell to the robot:

    `ssh pi@robotX`
> ⌨️ **Syntax:**  `ssh <username>@<hostname/IP address>`

1. Enter the robot's password

    You should see that the terminal lists the `pi` username and your robot's hostname, `robotX`. Any command you run in this shell will execute on the robot.
    <br>
1. Create a new package on the robot we can use to store scripts for this module. This package should be called 'module02' and should be in your student repo.  From the terminal ssh'd into the robot, type:

`cd ece387_robot_sp23-USERNAME/robot/`

`catkin_create_pkg module02 

1. Change directories to your new folder.

1. Print the working directory. You should get "/home/pi/robot_ws/src/ece387_robot_sp23-USERNAME/robot/"

1. You can type `exit` to close an SSH connection (but don't close the connection yet!).

> 📝️ **Note:** At times, there may be network issues and name resolution will fail. What this means is if you try to ping the robot from the master or vice versa using the hostname (e.g., `ping master0`) it will not work. However, it will work if you use the IP address (e.g., `ping 192.168.2.120`). To do this, you need the IP address of the machine you want to access. This IP address will change periodically. The easiest way to determine the current IP Address is, with the machine plugged into a monitor and keyboard, type `ip addr` in a terminal. This will list all of the network interfaces on the machine (such as *eth0* for ethernet and *wlan0* for wireless). You are looking for the `inet` field under `wlan0` (may be called `wlo1`). Now you can unplug the roobt from the monitor, ping the IP address to check connectivity, and then SSH into the robot.

The other remote tool you may want to take advantage of is SCP, which securely copies a file to a remote machine.

> ⌨️ **Syntax:**  `scp <src location> <username>@<hostname>:<dest location>`

For example, copy the `move_turtlebot_square.py` file to your robot by typing the following in a new terminal on the master:

`roscd module02/my_scripts`

`scp move_turtlebot_square.py pi@robot0:/home/pi/robot_ws/src/ece387_robot_sp23-USERNAME/robot/module02/src/`

> 📝️ **Note:** The destination uses the absolute path you printed earlier.

In your secure shell list the contents of your 'my_scripts' folder. You should see the `move_turtlebot_square.py` file. Check that it is executable. If it is not, make it executable.

Now let's move our TurtleBot using the script on the robot:

1. Run roscore on the master.

1. Run the simulated robot on the master.

1. SSH into your robot Raspberry Pi.

1. Run the move_turtlebot_square.py using the `rosrun` command on the robot.

You are now controlling your simulation (which is running on the master) remotely from the robot. In future lessons you will have nodes running on your robot to drive the robot and will control the robot remotely from the master.

## ROS
Below you will run ROS commands. The "!" character in the front allows us to run bash commands from Jupter and would **NOT** be used in the command line.

Accomplish the following on the master by adding the commands necessary below:

List all running nodes:


```python
!
```

List the active topics:


```
!
```

Display running nodes and communication between them:


```python
!
```

Exit the rqt_graph.

Show information about a the **/cmd_vel** topic such as what type of messages are sent over the topic and publishing and subscribing nodes.


```python
!
```

Display information about the message that is sent over the **/cmd_vel** topic.


```python
!
```

Display messages sent over the **/cmd_vel** topic:


```python
!
```

## Checkpoint
Once complete, get checked off by an instructor showing the output of each of the above.

> 📝️ **Note:** You will use all of the above ROS commands for each lab to write your lab reports. You could create a bash script to run these commands automatically ;)

Additionally, place screen captures showing each of the commands running or the windows they bring up into a folder within ../module02/Pictures on your master repo.  Then push the repo to get credit for this work.

## Summary
You have seen a lot of different Linux/ROS commands during this lesson but it only scratches the surface. There are tons of online resources available if you want to learn more. I recommend working through a few of these tutorials: http://www.ee.surrey.ac.uk/Teaching/Unix/. They are fairly quick and willl give you more insight into the tools we discussed.

## Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. In each of the notebooks reset the Jupter kernel and clear output. Now it is safe to exit out of this window. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.
