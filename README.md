# energy-ros2-cpp-python
Replication package of our paper that compares de energy efficiency of Python and C++ for ROS 2 projetcs.

## Monitoring the Energy Consumption

For monitoring the energy consumption, we are going to use the `energy-mon.py` script as superuser (root).

```
# python3 energy-mon.py <script> <language>
```

## Running Experiments

For running the experiments, we rely on [Experiment-Runner](https://github.com/S2-group/experiment-runner) (ER). Please, look at its documentation to get started.

All the experiment is defined in the [RunnerConfig.py](./exp-runner/RunnerConfig.py) file.

Install the project dependencies:

```bash
pip3 install -r requirements.txt
```

Configure the project path in the `setup.bash` file, and then run:

```bash
source setup.bash
python3 <ER folder> ./exp-runner/RunnerConfig.py
```

## TODO
- Remove unnecessary comments in the sorce code.
- Adapt the service and action packages to exit when the time is up (topic pub_sub are already set that way).
- Make all the packages to receive exactly the same pa1rameters or use additional parameters from the third position.
- For topic `pub_sub`, implement different message types.
- Measure the energy consumption.