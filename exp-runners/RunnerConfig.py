import os
import time
from EventManager.Models.RunnerEvents import RunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RunnerContext import RunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType
from ProgressManager.Output.OutputProcedure import OutputProcedure as output
from Plugins.System.Docker.DockerRunner import DockerRunner

from typing import Dict, List, Any, Optional
from pathlib import Path
from os.path import dirname, realpath

import subprocess

# from Plugins.Profilers import CodecarbonWrapper
# from Plugins.Profilers.CodecarbonWrapper import DataColumns as CCDataCols

# @CodecarbonWrapper.emission_tracker(
#     data_columns=[CCDataCols.EMISSIONS, CCDataCols.ENERGY_CONSUMED],
#     country_iso_code="BR" # your country code
# )

experiment_path = os.getenv('EXPERIMENT_PATH')

class RunnerConfig:
    ROOT_DIR = Path(dirname(realpath(__file__)))

    # ================================ USER SPECIFIC CONFIG ================================
    """The name of the experiment."""
    name:                       str             = "cpp_py_ros2"

    """The path in which Experiment Runner will create a folder with the name `self.name`, in order to store the
    results from this experiment. (Path does not need to exist - it will be created if necessary.)
    Output path defaults to the config file's path, inside the folder 'experiments'"""
    results_output_path:        Path             = ROOT_DIR / 'experiments'

    """Experiment operation type. Unless you manually want to initiate each run, use `OperationType.AUTO`."""
    operation_type:             OperationType   = OperationType.AUTO

    """The time Experiment Runner will wait after a run completes.
    This can be essential to accommodate for cooldown periods on some systems."""
    time_between_runs_in_ms:    int             = 1000

    docker_runner: DockerRunner

    def __init__(self):
        """Executes immediately after program start, on config load"""

        if not experiment_path:
            print('Please, set the experiemtn path: EXPERIMENT_PATH environment variable.')
            os._exit(0)
        else:
            self.docker_runner = DockerRunner()

            EventSubscriptionController.subscribe_to_multiple_events([
                (RunnerEvents.BEFORE_EXPERIMENT, self.before_experiment),
                (RunnerEvents.BEFORE_RUN       , self.before_run       ),
                (RunnerEvents.START_RUN        , self.start_run        ),
                (RunnerEvents.START_MEASUREMENT, self.start_measurement),
                (RunnerEvents.STOP_MEASUREMENT , self.stop_measurement ),
                (RunnerEvents.STOP_RUN         , self.stop_run         ),
                (RunnerEvents.POPULATE_RUN_DATA, self.populate_run_data),
                (RunnerEvents.AFTER_EXPERIMENT , self.after_experiment )
            ])
            self.run_table_model = None  # Initialized later

            output.console_log("Custom config loaded")

    def create_run_table_model(self) -> RunTableModel:
        """Create and return the run_table model here. A run_table is a List (rows) of tuples (columns),
        representing each run performed"""
        #package = FactorModel("ros_package", ['simple_publisher_subscriber', 'simple_service_client', 'action_tutorials'])
        package = FactorModel("ros_package", ['simple_publisher_subscriber'])
        #msg = FactorModel("msg_type", ['default', 'geometry_msg_twist', 'sensor_msg_pointcloud2', 'sensor_msg_image', 'sensor_msg_odometry']) # plain text, geometric data, different sensor data
        msg = FactorModel("msg_type", ['default'])
        #interval = FactorModel("msg_interval", [0.5, 1.0, 1.5])
        interval = FactorModel("msg_interval", [0.25, 0.5, 1.0])
        #num_client = FactorModel("num_clients", [1, 2, 3, 4, 5])
        num_clients = FactorModel("num_clients", [1, 2])
        language = FactorModel("language", ['py', 'cpp'])
        exec_time = FactorModel("exec_time", [10])
        self.run_table_model = RunTableModel(
            factors=[package, msg, interval, language, exec_time, num_clients],
            # exclude_variations=[
            #     {package: ['simple_service_client'], msg: ['geometry_msg_twist']},
            #     {package: ['simple_service_client'], msg: ['sensor_msg_pointcloud2']},
            #     {package: ['simple_service_client'], msg: ['sensor_msg_image']},
            #     {package: ['simple_service_client'], msg: ['sensor_msg_odometry']},
            #     #
            #     {package: ['action_tutorials'], msg: ['geometry_msg_twist']},
            #     {package: ['action_tutorials'], msg: ['sensor_msg_pointcloud2']},
            #     {package: ['action_tutorials'], msg: ['sensor_msg_image']},
            #     {package: ['action_tutorials'], msg: ['sensor_msg_odometry']}
            # ],
            repetitions = 2
        )
        return self.run_table_model

    def before_experiment(self) -> None:
        """Perform any activity required before starting the experiment here
        Invoked only once during the lifetime of the program."""

        output.console_log("Config.before_experiment() called!")

    def before_run(self) -> None:
        """Perform any activity required before starting a run.
        No context is available here as the run is not yet active (BEFORE RUN)"""

        output.console_log("Config.before_run() called!")
        output.console_log("Cleaning Docker...")
        command_cleaning = "./clean-containers.sh"
        subprocess.run(command_cleaning, shell=True)

    def start_run(self, context: RunnerContext) -> None:
        """Perform any activity required for starting the run here.
        For example, starting the target system to measure.
        Activities after starting the run should also be performed here."""

        output.console_log("Config.start_run() called!")

        output.console_log("Starting Docker containers...")
        output.console_log("Starting Server/Pub...")
        self.docker_runner.start_container('server', 1)

        variation = context.run_variation
        clients = variation['num_clients']
        output.console_log(f"Starting Clients from 1 to {clients}...")
        self.docker_runner.start_container('client', clients)

        package=variation['ros_package']
        match package:
            case 'simple_publisher_subscriber':
                pub='talker' # third parameter optional
                sub='listener' # third parameter optional
            case 'simple_service_client':
                pub='server'
                sub='client' # block the two parameters
            case 'action_tutorials':
                pass # make it compatible with the others
        interval=variation['msg_interval']
        language=variation['language']
        exec_time=variation['exec_time']
        
        containers = []
        commands = []

        output.console_log("Running Server/Pub command...")
        server_command = f"source /opt/ros/humble/setup.bash && source /projeto/install/setup.bash && ros2 run {package}_{language} {pub} {exec_time} {interval}"
        #threads.append = self.docker_runner.run_in_thread('docker_server_1',server_command)
        containers.append('docker_server_1')
        commands.append(server_command)

        output.console_log("Running Client(s)/Sub(s) command...")
        client_command = f"source /opt/ros/humble/setup.bash && source /projeto/install/setup.bash && ros2 run {package}_{language} {sub} {exec_time} {interval}"
        for c in range(clients):
            containers.append(f'docker_client_{c+1}')
            commands.append(client_command)

        commands_to_run = list(zip(containers, commands))

        self.docker_runner.execute_commands_in_parallel(commands_to_run)

        # Waiting for everything to close
        output.console_log("Waiting all the nodes to complete...")
        time.sleep(5)

    def start_measurement(self, context: RunnerContext) -> None:
        """Perform any activity required for starting measurements."""
        output.console_log("Config.start_measurement() called!")

    def stop_measurement(self, context: RunnerContext) -> None:
        """Perform any activity here required for stopping measurements."""

        output.console_log("Config.stop_measurement called!")

    def stop_run(self, context: RunnerContext) -> None:
        """Perform any activity here required for stopping the run.
        Activities after stopping the run should also be performed here."""

        output.console_log("Config.stop_run() called!")

    def populate_run_data(self, context: RunnerContext) -> Optional[Dict[str, Any]]:
        """Parse and process any measurement data here.
        You can also store the raw measurement data under `context.run_dir`
        Returns a dictionary with keys `self.run_table_model.data_columns` and their values populated"""

        output.console_log("Config.populate_run_data() called!")
        return None

    def after_experiment(self) -> None:
        """Perform any activity required after stopping the experiment here
        Invoked only once during the lifetime of the program."""

        output.console_log("Config.after_experiment() called!")
        output.console_log("Cooling down for 30 seconds...")
        time.sleep(30)

    # ================================ DO NOT ALTER BELOW THIS LINE ================================
    experiment_path:            Path             = None
