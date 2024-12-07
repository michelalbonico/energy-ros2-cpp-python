import sys
import os
import argparse
import pyRAPL
import csv

pyRAPL.setup()

parser = argparse.ArgumentParser(description='Energy Monitor - RAPL')
parser.add_argument("-p", "--package", help="ROS package to be used", nargs='?')
parser.add_argument("-a", "--algo", help="ROS algorithm inside a package", nargs='?')
parser.add_argument("-c", "--command", help="Command to run the algorithm in standalone mode", nargs='?')
parser.add_argument("-l", "--language", help="Algorithm language", nargs='?', choices=['cpp','python'])
parser.add_argument("-i", "--identifier", help="Measurement identifier", nargs='+')
parser.add_argument("-f", "--frequency", help="Msg Interval", nargs='?')
parser.add_argument("-t", "--timeout", help="Timeout", nargs='?')
parser.add_argument("-r", "--rapl", help="Enable RAPL", nargs='?')
args = parser.parse_args()

def execute_python_file():
   try:
      if (args.command):
         if (args.language == 'cpp'):
            command=args.command
         else:
            command='python3 '+args.command
      else:
         command='ros2 run '+args.package+' '+args.algo+' '+args.timeout+' '+args.frequency
      if args.rapl == '1':
         print("RAPL is set")
         meter = pyRAPL.Measurement(args.identifier[0])   
         meter.begin()
      else:
         print("RAPL is not set")
      os.system(f'{command}')
      if 'meter' in locals():
         meter.end()
         #print(meter.result)
         duration = int(meter.result.duration) / 1000000
         energy_microjoules = float(meter.result.pkg[0])
         energy_joules = energy_microjoules / 1e6
         average_power = float(energy_joules) / duration
         csv_filename = f'energy-{args.identifier[0]}.csv'
         with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Execution Time','Energy (microjoules)','Energy (joules)', 'Average Power (Watts)'])
            writer.writerow([duration, energy_microjoules, energy_joules, average_power])

   except FileNotFoundError:
      print(f"Error: The command '{command}' is not valid.")

def main():
    execute_python_file()

if __name__ == '__main__':
    main()