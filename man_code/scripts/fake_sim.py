import os.path
import sys
import pandas as pd

result_file = sys.argv[1] 
print(result_file, "result_file")


simulation_db = pd.DataFrame()
simulation_db = simulation_db.append({'Arm ID': 4,
                            'Point number': 0,
                            'Move duration': 0,
                            'Success': 0,
                            'Manipulability - mu': 0,
                            'Manipulability - jacobian': 0,
                            'Manipulability - cur pose': 0,
                            'Manipulability - roni': 0,
                            'Mid joint proximity': 0,
                            'Max Mid joint proximity':0,
                            'Sum Mid joint proximity':0,
                            'Sum Mid joint proximity- all joints':0},ignore_index=True)
print(simulation_db)
if(os.path.isfile(result_file)): 
	print("in if")
	simulation_db.to_csv(result_file, mode='a', index= True, header=False) ############ removed pd.Dataframe()
else:
	print("in else")
	print(simulation_db.to_csv(str(result_file), index= True))
	simulation_db.to_csv(result_file, index= True) ############ removed pd.Dataframe()
	df_saved_file = pd.read_csv(str(result_file))
	print(df_saved_file)