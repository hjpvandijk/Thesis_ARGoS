import os
import re
import time

def delete_frames(directory):
    # Regular expression to match the frame files
    frame_pattern = re.compile(r'^frame(\d{10})\.png$')

    for root, dirs, files in os.walk(directory):
        for file in files:
            match = frame_pattern.match(file)
            if match:
                frame_number = int(match.group(1))
                if (100 < frame_number < 6000) or (6002 < frame_number < 12000) or (frame_number > 12002):
                    file_path = os.path.join(root, file)
                    os.remove(file_path)
                    print(f"Deleted: {file_path}")


while(True):
    # Usage
    delete_frames('implementation_and_examples/experiment_results')
    #wait 30 sec
    time.sleep(30)