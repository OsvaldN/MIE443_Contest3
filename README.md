# MIE 443 - Contest 3 Deliverable

This is a submission for the design course **MIE443: Mechatronics Systems: Design and Integration** at the University of Toronto

Date: April 15, 2021
Group: 20

## Team:

|       Name       | Student Number |
|:----------------:|:--------------:|
|   Stefan Albers  |   1003476204   |
| Mithun Jothiravi |   1002321258   |
|   Osvald Nitski  |   1002456987   |
|    David Rolko   |   1003037420   |


## Running Group 20 Contest 2 Submission

Open seven terminals. Navigate to `catkin_ws` in each of them. Build the project files and source the appropriate scripts using the following command: `catkin_make && source devel/setup.zsh`

1. In terminal #1 go to catkin_ws/src/ and run `conda activate mie443`
    then change to catkin_ws/ and run `roslaunch mie443_contest3 turtlebot_world.launch world:=practice`

2. In terminal #2 run `roslaunch mie443_contest3 gmapping.launch`

3. In terminal #3 change to the project `src` directory and run `conda activate mie443; python victimLocator.py`

4. In terminal #4 change to the project `src` directory and run `conda activate mie443; python emotionClassifier.py --model mdl_best_bc.pth`

5. In terminal #5 run `roslaunch turtlebot_rviz_launchers view_navigation.launch`

6. In terminal #6 run `roslaunch mie443_contest3 contest3.launch`

**NOTE**: If exploration stops, re-launch contest 3 files by running the command again. Exploration related errors are outside of our problem scope as outlined [here.](https://piazza.com/class/kja4nxl2z3c6n1?cid=165)

7. In terminal #7 run `conda deactivate` then run `rosrun sound_play soundplay_node.py`

The output file containing the results of the emotions of the victims discovered can be found in `~/.ros` and is named `Output_file.txt`

## Previous Contest Repository:

MIE443 Contest 3 repository is [here](https://github.com/OsvaldN/MIE443_Contest3).
