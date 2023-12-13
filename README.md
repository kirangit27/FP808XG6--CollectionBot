# FP808XG6--CollectionBot
Repository for ENPM808X Final Project Fall 23 - Group 6

## FP808XG6 Badges
![CICD Workflow status](https://github.com/kirangit27/FP808XG6--CollectionBot/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/kirangit27/FP808XG6--CollectionBot/branch/Phase_1/graph/badge.svg)](https://codecov.io/gh/kirangit27/FP808XG6--CollectionBot) [![License: Apache](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)



## Overview
The Collection Robot is an important mechanism in any industrial automation process. It ensures autonmous kitting and assembling of parts in a timely fashion to increase overall throughput. The Agile Robots for Industrial Automation Challenge was developed by NIST that tests the ability of robots to perform a series of tasks in a dynamic environment. The tasks are designed to test the robot's ability to perform pick-and-place operations, assembly, and kitting in a simulated warehouse. In the context of ARIAC, agility refers to a robot's capacity for adaptation, efficiency, and autonomy in a dynamic manufacturing setting rather than just its speed or physical prowess. The competition acts as a trial run for the creation of algorithms that can be used in actual manufacturing environments. The ARIAC's competition platform standsout as the best simulating and testing ground for agile robots in dynamic industrial environments, which served as the perfect fit  for our solution demonstration.


### Team
(Group 6)
1. Kiran Patil
2. Vyshnav Achutan
3. Suryavardhan Reddy Chappidi

## Steps to Install ARIAC Competition Platform
Create a new ROS2 workspace:
```
    source /opt/ros/galactic/setup.bash
    mkdir -p ~/ariac_ws/src
    cd ~/ariac_ws

```

Clone the ARIAC repository:
```
    git clone https://github.com/usnistgov/ARIAC.git src/ariac

```
 Note: Always use the ariac2023 branch.

Install the dependencies:
```
    sudo apt install python3-rosdep
    sudo apt install openjdk-17-jdk
    sudo rosdep init
    rosdep update --include-eol-distros
    rosdep install --from-paths src -y --ignore-src
```

Build the ARIAC package:
```
    sudo apt install python3-colcon-common-extensions
    colcon build --packages-select ariac

```

Source the workspace:
```
    source install/setup.bash

```
For further installation steps follow the guide at : https://pages.nist.gov/ARIAC_docs/en/2023.5.0/getting_started/installation.html

### Phase_0 (Project Proposal)
| Document           |Link                                                                                         |
| ------------------------- | -------------------------------------------------------------------------------------------- |
| Project Proposal          | [link](https://github.com/kirangit27/FP808XG6--CollectionBot/blob/Phase_1/documents/ENPM808X%20-%20Final%20Project%20Proposal%20(Phase%200).pdf) |
| Quad Chart                | [link](https://github.com/kirangit27/FP808XG6--CollectionBot/blob/Phase_1/documents/QuadChart_Final.pdf) |
| UML Diagram               | [link](https://github.com/kirangit27/FP808XG6--CollectionBot/blob/master/UML/initial_uml/UML%20class.png) |



### Phase_1
| Document           |Link                                                                                         |
| ------------------------- | -------------------------------------------------------------------------------------------- |
| UML Diagram               | [link](https://github.com/kirangit27/FP808XG6--CollectionBot/blob/master/UML/revised__uml/UML%20Revised.png) |
| Proposal Video            | [link](https://drive.google.com/drive/folders/1-OMwoqAwp02GAUaVJTIPztD0t-gRjUfr) |
| AIP                       | [link](https://docs.google.com/spreadsheets/d/1FtwduI0UeLAkzt9SnpAClorMw1jcahy9qjM6wzhxDfg/edit#gid=0) |
| Sprint Planning & Review  | [link](https://docs.google.com/document/d/1ZsBr0pjIIcKHEvNL5tI986ulKs6IyMRJGUegzvP9S6Y/edit) |
