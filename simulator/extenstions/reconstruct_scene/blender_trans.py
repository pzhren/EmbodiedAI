
#cmd使用blender脚本需要前置 "D:\Program Files\Blender Foundation\Blender 4.1\blender.exe"（）
import sys
import bpy
import time
import argparse

input_path = "D://MMLM_Robot//Grasp_Nav//Simulator//refined_mesh//playroom//sugarfine_3Dgs7000_sdfestim02_sdfnorm02_level03_decim1000000_normalconsistency01_gaussperface1.obj"
output_path = "D://MMLM_Robot//Grasp_Nav//code\simulator//reconstruct_scene//scene_usd//playroom.usd"
obj_filePath = input_path


objs = bpy.data.objects
objs.remove(objs["Cube"], do_unlink=True)
    # bpy.data.objects.remove(obj, do_unlink=True)  

imported_object = bpy.ops.wm.obj_import(filepath=obj_filePath)
bpy.ops.render.render(write_still=True)
bpy.ops.wm.usd_export(filepath =output_path)

