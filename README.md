<h1>PID vs Non-PID Evolved Robot Brains for Platform Balancing</h1>

CS 396 -- Artificial Life, Final Project ("Science Option")
Chase Duvall

See gif.GIF for gif, see fitness plots below.

Video Link: <a href="https://youtu.be/d0VnmqV7Tjo">https://youtu.be/d0VnmqV7Tjo</a>

<h2>Background</h2>

PID ("Proportional, Integral, Derivative") control is a common method in engineering for maintaining a desired state in a system (e.g. the temperature of a room regulated by a thermostat, or the rate of output of one portion of an assembly line). The basic idea is that the current state ("Proportional"), the previous state added up over a certain time frame ("Integral"), and the rate of change of the current state ("Derivative") all play a role in deciding how the system should be controlled to achieve/maintain the goal state. For example, a thermostat that detects a room rising above a certain temperature (high proportional value) might engage cooling systems at a certain strength -- if it detects high temperatures for longer (integral), it may work harder, while if temperature is already dropping rapidly (derivative), it may cool with less intensity in order to save energy and not over-correct. This kind of control is useful in systems that are susceptible to outside forces, chaotic or unpredictable.

In this experiment, I try applying this means of control as part of evolved simulated robot creatures trying to balance on a platform that tilts back and forth sinusoidally. This is relevant to the study of artificial life as many real-life organisms (e.g. birds, or humans balancing on a beam) may have sense organs that can help tell their position/orientation (humans gyroscopically, or bird's through earth's magnetic field), and PID control may be one way of accomplishing an organism staying in one place/upright (or in the case of magnetic-field-based migration, one general region).

<h2>Repo Code Usage</h2>

Running simulations: Running "python3 main.py BODYEV PID" will pass through several simulated "runs", each with a different random seed. This will not show any GUI to the program user (for that, see next paragraph). As-is, 5 runs will occur, each with 50 generations of 10 simulated creatures, simulated for 10 virtual seconds (these numbers can be modified in constants.py). Saved graphs include "lineageGraph" png files, where the number at the end of the filename is the seed of a run and the graph is the fitness by lineage for that run, and "fitnessGraph" png files, which compare the fitness of the fittest robot at each generation across all runs of the simulation, with the number at the end of the filename being the random seed of Run 0. Running "python3 main.py X1 X2 ... Xn", where each Xi is a random seed, will perform a run with that sequence of seeds (caution: it may overwrite previous saved fitnessGraph files if the first seed was seed 0 of a previous simulation). Running main.py will produce JSON files that save information about the first and last-evolved generation of every run.

Viewing simulations: Running "python3 view.py popToShow viewLen pid filename", where popToShow is replaced by a number of creatures to show (no more than the population size of file filename), viewLen is replaced by a number of seconds to view the simulation (which can be larger than the original simulation length), pid is either "True" or "False" and must correspond to whether filename stores results from a PID or non-PID simulation, and filename is the name of a "firstGen" or "lastGen" JSON file, will show creatures from the first or last generation of a previously-simulated run. Current JSON files are from the experiment described below.

<h2>Simulator Explanation (Control Scenario)</h2>

<b>Control Scenario Body Generation/Evolution/Morphospace</b>

The methods for generating/evolving robot bodies are shown below:

(See third image below for brain generation/evolution method)

![new_crop](https://user-images.githubusercontent.com/57238295/224511891-ebe0ac72-63e6-4b7d-b949-793e924c6903.jpg)

![Diagram2](https://user-images.githubusercontent.com/57238295/221744777-169ac7fd-ec03-434c-86c9-dd281455203f.jpg)

Initial robots are generated with a random number of rectangular prisms to their body (3-5), a 50% chance of each prism having a sensor (segments with sensors are green, ones without are blue) and random shapes for each prism (0.5-4 units long, wide and tall). Each pair of connected prisms are joined by a single joint pointed in the positive x, y or z direction (with equal probability). The creatures are generated by first creating an initial prism, then iteratively choosing a random prism that has already been created, a random face on that prism, and a random location of that face for the joint. Then, a random joint direction/new prism size are determined, and the new prism is created flush against the existing prism, centered on the joint point (with no initial rotation). If at any point during this process a prism would overlap with existing prisms or surpass a total creature height limit (currently set at 3 units as self.maxHeight in solution.py), the prism is discarded and a new one is calculated (as many times as needed to find a valid prism). In this way, robot bodies are generated to fill space in all 3 dimensions (as prisms are added to any face of existing prism).

The robot bodies are evolved in the following way: Each generation, there is a 30% chance of the the program attempting to mutate the body. If an attempt at mutation is made, there is a 10% chance of adding a prism, in which case a new prism is added in the same way prisms are added when the body is initialized (choose a random existing prism, face, offsets on that face, dimensions of the new prism and joint direction, repeating the process until a solution is found that satisfies the non-overlap and height limit constraints). Otherwise (with a 90% chance), the program attempts to remove a cube, deleting a random cube, its parent joint and all of its "descendant" cubes and joints -- it will simply abort this procedure if less than 2 cubes remain after deletion.

All in all, the body morphospace of this program includes a wide variety of creatures composed of rectangular prisms (including those which overlap with themselves), with parts initially flush against one another, joints being at the center of at least one of the two parts they connect, joints in 1 of 3 cardinal directions and all joints being simple revolute joints. It does not include creatures with other shapes of body parts, parts not flush against each other on initialization, joints with neither of their parts centered on the joint, joints in a direction other than the cardinal 3 or ball-and-socket or sliding joints.

<b>Control Scenario Brain Generation/Evolution/Morphospace</b>

The neural network "brain" of the creature is initially generated by creating a "sensor neuron" for each sensor prism and a "motor neuron" for each joint -- the layer of sensor neurons is fully connected to a hidden layer of neurons (equal in size to the sensor neuron layer), which is then fully connected to the layer of motor neurons, and weights are initialized randomly as a floating point number between -1 and 1.

The brain is evolved by randomizing a single synapse weight in the brain in one of the 2 layers (after body mutation has occurred, if it occurrs).

Regarding the brain morphospace, only brains with 1 hidden layers and 2 fully-connected groups of synapses "sandwiching" it are possible. Because of these fully-connected synapses, a sensor on one body can affect a motor on the other side (or anywhere on the body).

<b>Evolutionary Selection</b>

Every generation (whether the body is mutated or not), for each member of the robot population, the fitness of the mutated robot is compared to its parent, and the fitter creature continues evolving. This makes the method of selection a "Parallel Hill Climbers" -- fellow members of the population do not directly interact, and the program tries making one change (or one brain and one body change) to robots at a time to check for improvement.

<h2>Experiment</h2>

<b>Hypothesis</b>

Adding 9 "PID neurons" (in addition to body sensor neurons), with a proportional, integral and derivative neuron for each of the 3 spatial axes, to the input layer of the robot's neural network "brain" will lead to improved performance at balancing on the platform, as measured by the euclidean distance of the robot from its starting point after 10 simulated seconds.

<b>Methods</b>

![pid_brain](https://user-images.githubusercontent.com/57238295/224202195-ebbd39a4-eb52-4a31-8564-01577656966b.jpg)

For the experimental scenario, the Pyrosim simulated robot library was modified to allow the creation of "controlled neurons" whose values are set manually by each instance of the ROBOT class, and the ROBOT class was modified to create the 9 PID neurons. For each spatial dimension, a proportional neuron was added whose value was set to the robot's current position in that dimension at each frame, an integral neuron was added whose value was set to the average of past positions in that dimension over the past 60 frames, and a derivative neuron was added whose value was set to the robot's change in position in that dimension since last frame. The hidden layer of the brain was also enlarged to correspond to the new size of the input layers.

To isolate the variable of interest, a "PID" or "NOPID" flag is taken on the command line for main.py, which is then passed down into the s

For both the control and experimental scenario, 5 evolutionary runs with a population size of 10 and simulations of 10 simulated seconds were performed. Because of the experimenter's major time constraints, only 50 generations were calculated for each run.

<b>Results</b>

Without PID:

![fitnessGraph_NOPID_3924284438796659210](https://user-images.githubusercontent.com/57238295/224202303-8dad7323-3aef-4bdc-8fc0-87caff926498.png)

With PID:

![fitnessGraph_PID_7499076605079159191](https://user-images.githubusercontent.com/57238295/224202325-f6f185c9-c894-4da5-93ba-9044991aa706.png)

Lineage example (No PID, run 0):

![lineage_nopid](https://user-images.githubusercontent.com/57238295/225185875-7407d6d0-f228-4364-a7e3-1e92fea11417.png)

Lineage example (PID, run 0):

![lineage_pid](https://user-images.githubusercontent.com/57238295/225185913-872bb6a6-9f95-42ca-9d51-a4333ce0aa69.png)

<b>Discussion and Conclusion</b>

From the data above, it appears that adding the 9 PID neurons had a noticable positive effect on robots' performance in staying near the origin while the platform oscillated, with fitness (negative distance to origin after 10 seconds) tending to both rise faster and be absolutely higher after 50 generations). This implies that PID can indeed be usefully employed to maintain homeostasis for an organism in a shifting external environment, just as it can be used to maintain a mechanical control system's internal state. With more time, this simulation could be run for a larger number of generations in order to quantify the magnitude of the effect across longer time scales.

<b>Resources Used</b>

Pyrosim: github.com/jbongard/pyrosim

r/ludobots: reddit.com/r/ludobots/wiki/installation/

CS 396 -- Artificial Life, taught by Sam Kriegman, at Northwestern University
