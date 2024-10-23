import sys
import os
import argparse
import pyRAPL

pyRAPL.setup()

parser = argparse.ArgumentParser(description='Energy Monitor - RAPL')
parser.add_argument("-p", "--package", help="ROS package to be used", nargs='?')
parser.add_argument("-a", "--algo", help="ROS algorithm inside a package", nargs='?')
parser.add_argument("-c", "--command", help="Command to run the algorithm in standalone mode", nargs='?')
parser.add_argument("-l", "--language", help="Algorithm language", nargs='?', choices=['cpp','python'])
parser.add_argument("-i", "--identifier", help="Measurement identifier", nargs='+')
parser.add_argument("-d", "--destination", help="Destination folder", nargs='?')
args = parser.parse_args()

destination_folder = args.destination
csv_output = pyRAPL.outputs.CSVOutput(f'{destination_folder}/result.csv')
meter = pyRAPL.Measurement(args.identifier)

def execute_python_file():
   try:
      if (args.command):
         if (args.language == 'cpp'):
            command=args.command
         else:
            command='python3 '+args.command
      else:
         command='ros2 run '+args.package+' '+args.algo
      meter.begin()
      os.system(f'{command}')
      meter.end()
      meter.export(csv_output)
   except FileNotFoundError:
      print(f"Error: The command '{command}' is not valid.")

def main():
    execute_python_file()

if __name__ == '__main__':
    main()
    csv_output.save()