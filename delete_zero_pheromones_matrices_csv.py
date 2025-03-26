import pandas as pd
import argparse

def filter_csv(input_file, output_file, column_name):
    # Read the CSV file
    df = pd.read_csv(input_file)
    
    # Filter out rows where the specified column has value 0
    df_filtered = df[df[column_name] != 0]
    
    # Save the modified CSV
    df_filtered.to_csv(output_file, index=False)
    print(f"Filtered CSV saved as {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Remove rows with 0 in a specified column from a CSV file.")
    parser.add_argument("input_file", help="Path to the input CSV file")
    parser.add_argument("output_file", help="Path to save the modified CSV file")
    parser.add_argument("column_name", help="Column to check for 0 values")
    
    args = parser.parse_args()
    
    filter_csv(args.input_file, args.output_file, args.column_name)
