#!/usr/bin/env python
import glob
import pandas as pd
import os
import re

list_of_problematic = []
for file_or_dir in glob.glob("/home/ar1/Desktop/problematic_link_families/*"):
        if os.path.isfile(file_or_dir):
                #arm_name = re.sub('\.urdf$', '', file_or_dir)
                head_tail = os.path.split(file_or_dir)
                file_with_extension = head_tail[1]
                filename, file_extension = os.path.splitext(file_with_extension)
                arm_name = re.sub('\.urdf$', '', filename)
                list_of_problematic.append(arm_name)
my_data = pd.DataFrame(list_of_problematic)
my_data.to_csv("problematic_link_families6dof.csv", index = False, header= False)