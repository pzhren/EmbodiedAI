# Embodied AI Simulator



## test version

You can run `\simulator\tests\isaac-sim_test.py` for easy test.



## TODO LIST

- [ ] API Documents

- [ ] Run Base Module

- [ ] More Module and Test


## Install

1. [Install Isaac Sim 4.0](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/index.html)

2. Windows or Linux Set up (using conda)
#### Windows 
```
\simulator\scripts\set_up.bat
```
#### Linux
```
\simulator\scripts\set_up.sh
```
Then pip install packages for you conda environment
```
conda activate isaacsim
cd \simulator
pip install -r requirements.txt
```
3. You can also try [pip install isaac-sim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#install-isaac-sim-using-pip)
```
conda create -n isaacsim python=3.10
pip install isaacsim==4.0.0.0 --extra-index-url https://pypi.nvidia.com
cd \simulator
pip install -r requirements.txt
```

4. Test the installation
```
python \simulator\tests\isaac-sim_test.py
```
It will create a playroom contains a robot, you can use keyboard to control the robot.
play_room.usd can be found in [here](https://embodied-ai-lab.oss-cn-beijing.aliyuncs.com/refined_mesh/playroom/playroom.usd?Expires=1722360664&OSSAccessKeyId=TMP.3KjfBpJKQjQYtou6pEXUxQosAMKsqpC1ZD36aaquhRyFvThQChyZ2FgVaUSq5GmbBVL7Xpa57kWTWw1dw2dx56mYFYFzBu&Signature=gQL6%2FJLkz71%2BYsL0dtGWOP6K%2FBA%3D)

#### Docker Set up
will coming soon.


