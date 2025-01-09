import re

# Function to process the input file and extract coordinate pairs
def parse_coordinates(input_file):
    with open(input_file, 'r') as file:
        lines = file.readlines()

    # Regular expression to match the coordinate pairs
    # coord_pattern = re.compile(r"x = {double} (-?\d+\.\d+)\ny = {double} (-?\d+\.\d+)")
    coord_pattern_x = re.compile(r"x = {double} (-?\d+\.\d+)")
    coord_pattern_y = re.compile(r"y = {double} (-?\d+\.\d+)")

    # List to store the extracted coordinates
    coordinates = []

    # Iterate over the lines and extract the coordinates
    for i in range(0, len(lines), 2):
        # Extract first and second coordinates
        line1 = lines[i].strip()
        line2 = lines[i+1].strip()
        first_match = coord_pattern_x.match(lines[i].strip())
        second_match = coord_pattern_y.match(lines[i+1].strip())

        if first_match and second_match:
            first_coord = (float(first_match.group(1)), float(first_match.group(2)))
            second_coord = (float(second_match.group(1)), float(second_match.group(2)))
            coordinates.append((first_coord, second_coord))

    return coordinates

# Function to write coordinates to a new Python file
def write_coordinates_to_python(coordinates, output_file):
    with open(output_file, 'w') as file:
        file.write("coordinates = [\n")
        for first, second in coordinates:
            file.write(f"    ({first}, {second}),\n")
        file.write("]\n")

# Main function to run the transformation
def main(input_file, output_file):
    coordinates = parse_coordinates(input_file)
    write_coordinates_to_python(coordinates, output_file)
    print(f"Coordinates have been written to {output_file}")

# Example usage
input_file = 'route.txt'  # Replace with the path to your input file
output_file = 'coordinates_output.py'  # Replace with the desired output file path
main(input_file, output_file)
