Assignment 7 for the Northwestern course CS 396 -- Artificial Life. Running "python3 main.py" will show a sequence of randomly-generate artificial creatures, each with a random number of rectangular prisms to their body (3-10), a 50% chance of each prism having a sensor (segments with sensors are green, ones without are blue) and random shapes for each prism (0.5-4 units long, wide and tall).

Each pair of connected prisms are joined by a single joint pointed in the positive x, y or z direction (with equal probability). The creatures are generated by first creating an initial prism, then iteratively choosing a random prism that has already been created, a random face on that prism, and a random location of that face for the joint. Then, a random joint direction/new prism size are determined, and the new prism is created flush against the existing prism, centered on the joint point (with no initial rotation).

The neural network "brain" of the creature is generated by creating a "sensor neuron" for each sensor prism and a "motor neuron" for each joint -- the layer of sensor neurons is fully connected to the layer of motor neurons, and weights are initialized randomly as a floating point number between -1 and 1.

All in all, the morphospace of this program includes a wide variety of creatures composed of rectangular prisms (including those which overlap with themselves), with parts initially flush against one another, joints being at the center of at least one of the two parts they connect, joints in 1 of 3 cardinal directions and all joints being simple revolute joints. It does not include creatures with other shapes of body parts, parts not flush against each other on initialization, joints with neither of their parts centered on the joint, joints in a direction other than the cardinal 3 or ball-and-socket or sliding joints. Only fully-connected brains with no hidden layers are possible (sensors are directly connected to motors, with every sensor connected to every motor with some random weight, though some of these weights could be close to zero). Because the brain is fully connected, a sensor on one body can affect a motor on the other side (or anywhere on the body).