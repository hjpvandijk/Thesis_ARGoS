import os
import csv
from zipfile import ZipFile
from functools import lru_cache
import sys

@lru_cache(maxsize=10)  # Adjust cache size as needed
def get_zip(zip_path):
    return ZipFile(zip_path, "r")

print("Getting all files in the usb drive")
usb_drive = '/media/hugo/Philips_main/CLARE/'
zipfiles = []
completed_per_zip = []
# for all files in the usb drive
for file in os.listdir(usb_drive):
    if file.endswith(".zip"):
        if file =='experiment_results_afternoon_11-03.zip':
            continue
        zip_file = usb_drive + file
        zipfiles.append(zip_file)

for zip_file in zipfiles:
    found = False
    zipfilename = zip_file.split('/')[-1]
    completed_in_zipfile = f"completed_in_zip/perzip/completed_experiments_unique_{map}_{zipfilename}.csv"
    if not os.path.exists(completed_in_zipfile):
        continue
    reader = read_file(completed_in_zipfile)
    for row in reader:
        if exp == row[0]:
            experiment_is_in_zipfile = zip_file
            found = True
            break
    if found:
        break
if not found:
    print("not found:", exp)
    continue
zf = get_zip(experiment_is_in_zipfile)