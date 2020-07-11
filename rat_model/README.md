
# my changes 

## rat_model2.py

A raster and an histogram can be plotted in the function buildSimConfig
The average of each opulation can be retrieved in the function adjustNetworkAndCommunicateViaROS
A stimulation is received through ros 
A k-mean function has been implemented to determine in which cluster is the signal


## rat_model3.py
A stimulation is received through ros 

Instead of a kmean cluster, an ANN is used.
The probability of having PD, wether or not the model has PD and the frequency rate, ISI mean and ISI std of the model are saved in a csv file.
The extracted data for the ANN comes from world/data/rat_model/pd_stimu_data_ANN.csv
The saved data is saved in world/data/rat_model/pd_stimu.csv

Everything is set and launched with 

```bash
roslaunch rat_model testbash.launch
```
(which launch the bashcommand.sh in script -to set the environment automatically-, be wary, to use you need to change the path to your repository in there)


# Jhielson presentation of the rat_model

# A biophysical model inspired by the brain structures of rats 

The BG-C-T system was built using NetPyNE library and can be simulated on NEURON. 

You can also use ROS to communicate to other applications and share data via topics and services.  

## Installation

System running Ubuntu 18.04. Follow instructions on linked websites to install required tools.

1. Python 2.7 
2. [ROS Melodic](http://wiki.ros.org/melodic)
3. Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) if you don't have one already
4. Git clone this package into your ROS workspace
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/jhielson/rat_model.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
5. NEURON
    - NEURON installation is the most complex and involves building NEURON from source, please follow the tutorial carefully.
    - This guide is adapted from [this tutorial](https://neurojustas.com/2018/03/27/tutorial-installing-neuron-simulator-with-python-on-ubuntu-linux/), if you have problems you can try following this.
    - Open a terminal, change directory to your clone of this repo, for me its `cd ~/catkin_ws/src/rat_model`
    - Install python dependencies with `pip install scipy numpy matplotlib cython mpi4py neuronpy`
    - Install neurons non python dependencies:
        - `sudo apt-get update`
        - `sudo apt-get install libx11-dev libxt-dev gnome-devel`
        - `sudo apt-get install mpich`
        - `sudo apt-get install libncurses-dev xfonts-100dpi`
        - logout and log back in?
    - Download neuron source code and extract to `~/neuron` by running `bash installation_scripts/downloadNeuron.sh`
    - Build the neuron source code by running `bash installation_scripts/buildNeuron.sh`
    - Make sure your built version of NEURON is visible to tools by adding the line `export PATH=$HOME/neuron/iv/x86_64/bin:$HOME/neuron/nrn/x86_64/bin:$PATH` to your `~/.bashrc` file
    - Run `source ~/.bashrc` (or log out and log back in to make the previous change apply)
    - Finally install the neuron python package
        - `cd ~/neuron/nrn/src/nrnpython/`
        - `sudo python setup.py install`
    - You can verify the setup was successful by starting python `python` and trying to import the neuron package `import neuron`. If there are problems then an error will be raised, otherwise a copyright notice will be printed.
6. [NetPyNE](http://netpyne.org/install.html#install-only-netpyne)
7. load the project neuron modules with installed NEURON
```
$ cd ~/catkin_ws/src/rat_model/scripts/
$ nrnivmodl
```

## Running the simulation

1. Start ROS by running `roscore` in a second terminal
2. Make sure the rat_model package is on the `$ROS_PACKAGE_PATH` by running `source devel/setup.bash`
3. Run the simulation with
```
cd ~/catkin_ws/src/rat_model/scripts/
rosrun rat_model model.py
```

2.1 If you get the following error
```
AttributeError: 'hoc.HocObject' object has no attribute 'Izhi2003b'
```
Try running the following command again
```
$ cd ~/catkin_ws/src/rat_model/scripts/
$ nrnivmodl
```

## Tutorial

This model was designed inspired by the following work:

*Kumaravelu K, Brocker DT, Grill WM. A biophysical model of the cortex-basal ganglia-thalamus network in the 6-OHDA lesioned rat model of Parkinson's disease. J Comput Neurosci. 2016;40(2):207‐229. doi:10.1007/s10827-016-0593-9*

You can simulate the network, compare the neurons' action potential activities, update their stimulus and do many other adjustments.  

***

The template file on which you will build your work is located in the folder 'scripts'. The name of the file is 'model' and it was coded in Python.

The following instructions were organized in different parts:

1. Main scope

There are three important commands in this part of the code.

- Initializing ROS: you can specify the name of your node. It must be unique in a running system. 
```python
    # ROS: create node
    rospy.init_node('neural_model_rat', anonymous=False)
```
- Creating the network: you need to specify the state of the model (1 to PD and 0 to Healthy) and the duration (ms) of the simulation. There are other parameters but those are more important initially.
```python
    # Create network
    pd = 0     
    network = Network(t_sim = 90000, has_pd=pd)
```
- Then, you can start the simulation. The interval value represents the size of the window on which you want to stop the simulation to do the analysis.
```python
    # Simulate 
    network.simulate(pd, dt = 0.1, lfp = False, interval = 1000)
```

2. Breaking the simulation in fixed intervals

After each interval, the following function is called:
```python
    def adjustNetworkAndCommunicateViaROS(self,time):
        # Check time
        self.current_interval = time

        # Collect Data 
        sim.gatherData()
        
        # Do the analyses
        # ADD CODE HERE

        # Send the new information via ROS
        # ADD CODE HERE
```
There, you can add your analysis and send all the new information directly to other applications.

3. Online adjustments 

You might need to change the intesity of some stimulus that are aplied directly to some neurons during the execution of an experiment. For instance, for thalamic neurons, you can update the intensisty of the current by the following command:

```python
sim.net.modifyStims({'conds':{'source':'Input_th'},'cellConds':{'pop':'TH'},'amp':0.0018})
```

## NetPyNE

If you would like to look for new commands to generate plots or to even manipulate the model, please check the documentation and tutorials available on NetPyNE webpage (http://netpyne.org/tutorial.html).

## NAO robot + ROS

Using topics and service, you can share information between this model and your robot's controller. For more information about simulating NAO robot on Webots and incorporating ROS on it, please check the following repository (https://github.com/jhielson/neurorobotics_computational_environment). 


