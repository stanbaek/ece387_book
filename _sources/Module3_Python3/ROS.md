# ICE3: ROS

## ROS
Below you will see some ROS commands. The "!" character in the front allows us to run bash commands from Jupyter and would **NOT** be used in the command line.

With the user and computer nodes running execute the below commands.

List the active topics:


```bash
rostopic list
```

Display running nodes and communication between them:


```bash
rqt_graph
```

Exit the rqt_graph and then list all running nodes:

```bash
rosnode list
```

Show information about a the **/computer_choice** topic such as what type of messages are sent over the topic and publishing and subscribing nodes:

```bash
rostopic info /computer_choice
```

Display information about the message that is sent over the **/computer** topic:

```bash
rostopic type /computer_choice | rosmsg show
```

Display messages sent over the **/user_choice** topic:

```bash
rostopic echo /user_choice
```

In the Module3_Python3 notebook, reset the Jupter kernel and clear output: At the top menu bar select "Kernel" and "Restart & Clear Output" (you do not need to restart the computer player). Make sure to rerun the code blocks to import modules, build the class, and create the main. When you select 'Rock', 'Paper', or 'Scissors' you should see the message echoed above.

## Cleanup
In each of the notebooks reset the Jupter kernel and clear output. Close each notebook. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `Jupter-notebook` in. Select 'y'. Kill all processes in your terminals (ctrl+c) and ensure roscore is terminated before moving on to the ICE.

You should now have a better understanding of how to utilize ROS and Python. To learn more about the Python style guide and standards, visit [PEP 8 -- Style Guid for Python Code](https://www.python.org/dev/peps/pep-0008/#class-names).


## Deliverables

Below you will run ROS commands. The "!" character in the front allows us to run bash commands from Jupyter and would **NOT** be used in the command line.

With the client and server nodes running execute the below commands.

List all running nodes:


```python

```

List the active topics:


```python
 
```

Display running nodes and communication between them:


```python
 
```

Exit the rqt_graph.

Show information about a the **/client** topic such as what type of messages are sent over the topic and publishing and subscribing nodes.


```python
 
```

Display information about the message that is sent over the **/client** topic.


```python

```

Display messages sent over the **/client** topic:


```python

```

In the ICE3_Client notebook when you send a message you should see the message echoed above.

## Checkpoint
Once complete, take the necessary screen shots of all the items above functioning. Upload those screenshots to your repo in a Module03 folder within the master folder of your repo.

## Cleanup
In each of the notebooks reset the Jupter kernel and clear output. Close each notebook. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.
