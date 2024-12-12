# energy-ros2-cpp-python
Replication package of our paper that compares de energy efficiency of Python and C++ for ROS 2 projetcs. Document under supervision at `Frontiers Robotics and AI`.

## How to cite us

TBA

## Algoritm Complexity Measurement

We have a local SonarQube deployment to check the algorithm measurements.

To run it in you machine, follow the steps:

```bash
bash sonarqube.sh
bash sonarqube-scanner.sh
```

In your browser, navigate to `http://localhost:9000` and use the default username `admin`, which is also the password (you are requested to change it after loggin in).

Since we need special wrappers for `Cpp` algorithms, which are not trivial to set up, you can also use the [`lizard`](https://ascl.net/1906.011) tool for both, `Python` and `Cpp`.

## Running Experiments

For running the experiments, we rely on [Experiment-Runner](https://github.com/S2-group/experiment-runner) (ER). Please, look at its documentation to get started.

All the experiment is defined in the [RunnerConfig.py](./exp-runner/RunnerConfig.py) file.

Install the project dependencies:

```bash
pip3 install -r requirements.txt
```

Install `PowerJoular` for energy consumption measurements: [Documentation WebSite](https://joular.github.io/powerjoular/guide/installation.html).

Configure the right paths in the `setup.bash` file, and then run:

```bash
source setup.bash
python3 $EXPERIMENT_RUNNER_PATH ./exp_runner/RunnerConfig-pubsub.py &&
python3 $EXPERIMENT_RUNNER_PATH ./exp_runner/RunnerConfig-service.py &&
python3 $EXPERIMENT_RUNNER_PATH ./exp_runner/RunnerConfig-action.py
```

We were running the experiment incrementally as we finished programming the algorithms, soon we will provida a single execution file.

## Statistical Analysis and Graph Generation

Change directory to `data-analys` folder:

```bash
cd data-analisys/
```

Run the statistical tests:
```bash
python3 statistical_tests.py
```

All the graphs are saved in the `graphs` folder, with the following subfolders:

```
- no_transf-no_out/  Original data.
- no_transf-out/     Data transformed.
- transf-no_out/     Data without outliers.
- tranf-out/         Data transformed and without outliers.
```