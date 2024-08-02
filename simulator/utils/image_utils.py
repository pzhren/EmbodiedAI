import os
import cv2

def img2video(file_dir:str, video_path:str, img_size:tuple=(1280,720), fps:int=5):
    list = []
    for root ,dirs, files in os.walk(file_dir):
        for file in files:
            list.append(file)     
    video = cv2.VideoWriter(video_path,cv2.VideoWriter_fourcc(*'MJPG'),fps,img_size)

    for i in range(1,len(list)):
        img = cv2.imread(file_dir+list[i-1])     
        img = cv2.resize(img,img_size)
        video.write(img)
        
    video.release()