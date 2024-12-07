pkill python3
cd /home/michel/Documents/GitHub/energy-ros2-cpp-python/
rm -f *.csv
rm -f energy*
source setup.bash &&
source .venv/bin/activate &&
python3 $EXPERIMENT_RUNNER_PATH exp_runners/RunnerConfig-pubsub.py
