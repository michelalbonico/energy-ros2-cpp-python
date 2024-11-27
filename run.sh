source setup.bash

for i in {1..450}:
    yes | python3 $EXPERIMENT_RUNNER_PATH exp_runners/RunnerConfig-service.py