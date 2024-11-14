# energy-ros2-cpp-python
Replication package of our paper that compares de energy efficiency of Python and C++ for ROS 2 projetcs.

## Algoritm Measurement

We have a local SonarQube deployment to check the algorithm measurements.

To run it in you machine, follow the steps:

```bash
bash sonarqube.sh
bash sonarqube-scanner.sh
```

In your browser, navigate to `http://localhost:9000` and use the default username `admin`, which is also the password (you are requested to change it after loggin in).

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
- Remove unnecessary comments from the sorce code.
- Adapt the service and action packages to exit when the time is up (topic pub_sub are already set that way).
- Make all the packages to receive exactly the same pa1rameters or use additional parameters from the third position.
- The `action` package must send coordinates (sleep time * x) to the turtlesim, and from time to time (sleep time), get the feedback. 
- For topic `pub_sub`, implement different message types.
- Measure the energy consumption.