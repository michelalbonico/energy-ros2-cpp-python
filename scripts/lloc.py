import argparse

def count_cpp_lloc(file_path):
    lloc = 0
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            # Ignore blank lines, comments, and preprocessor directives
            if line and not line.startswith(("//", "/*", "#")):
                lloc += 1
    return lloc

# Set up argument parsing
parser = argparse.ArgumentParser(description='Count Logical Lines of Code (LLOC) in a C++ file.')
parser.add_argument('file', help='Path to the C++ file')

# Parse the arguments
args = parser.parse_args()

# Count LLOC in the provided C++ file
lloc = count_cpp_lloc(args.file)
print(f"Logical Lines of Code in {args.file}: {lloc}")
