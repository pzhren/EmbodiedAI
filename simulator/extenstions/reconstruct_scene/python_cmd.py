import os, subprocess

blender_path = "D:\Program Files\Blender Foundation\Blender 4.1//blender.exe"
python_path = "D:\MMLM_Robot\Grasp_Nav\code\simulator//reconstruct_scene//blender_trans.py"

cmd = '{} -b -P {} -i {} -o {}'.format(blender_path, python_path, input_path, out_path)
subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
