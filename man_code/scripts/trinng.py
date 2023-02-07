import os
import time
import moveit_msgs.msg
import geometry_msgs.msg
import pandas as pd
import math
import re
from ros import Ros, MoveGroupPythonInterface, UrdfClass
from datetime import datetime
from os import environ, listdir, path, mkdir, rename
import numpy as np
from itertools import product
from time import sleep
from rospy import init_node
from rosnode import get_node_names
import getpass
import sys
import json
import rospy
import csv
import random


links = pd.read_csv('10_link_configurations.csv')
links = links.drop(columns='Unnamed: 0')

l = links.values.tolist()

final = []
for x in l:
	ls = []
	for y in x:
		ls.append(str(y))
	final.append(ls)

print(l)
print(final)