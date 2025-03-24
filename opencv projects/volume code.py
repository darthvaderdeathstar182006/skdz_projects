import cv2
import time
import numpy as np
import handtrackmod as htm
import math
import volumemod as vom
import subprocess

wCam , hCam = 640, 480   #heaight and width of the pop up window when u run the program
#####################################
cap = cv2.VideoCapture(0)
cap.set(3, wCam)#prop id 3 is width of camera and 4 is its height
cap.set(4, hCam)
ptime = 0
detect = htm.handetect(maxhandz=1, trackconfident=0.75, detectconfident=0.75)
vm = vom.volumecontrol()

while True:
    success, img = cap.read()
    img = detect.findHands(img)
    lmlist = detect.findpositionpoint(img, draw = False)
    if len(lmlist) != 0:
        #print(lmlist[4],lmlist[8])#get position of point 4 and 8 on hand
        detect.drawcircle(img, 8)#x1, y1 = lmlist[4][1], lmlist[4][2]
        detect.drawcircle(img, 4)  # cv2.circle(img,(x1,y1), 15, (0,0,255), cv2.FILLED)
        x1, y1 = lmlist[4][1], lmlist[4][2]
        x2, y2 = lmlist[8][1], lmlist[8][2]
        ####draw line between the point
        cv2.line(img, (x1,y1),(x2,y2), (0,255,0), 3)
        ####find its center
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)
        # GIVES LENGTH OF LINE
        length = math.hypot(x2-x1, y2-y1)#GIVES LENGTH OF LINE
        #print(int(length))
        vol = np.interp(length, [20,200], [0,100])
        print(int(vol))
        vm.controlvol(vol)
        cv2.rectangle(img,(50,150),(85,400),(0,255,0), 3)
        bar = np.interp(int(vol), [0,100], [400,150])
        cv2.rectangle(img, (50, int(bar)), (85, 400), (0, 255, 0), cv2.FILLED)
        cv2.putText(img, f'{int(vol)}%', (45,420), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0), 3)

        if length < 50:
            cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
            #vm.decvol()


    ctime = time.time()
    fps = 1/(ctime-ptime)
    ptime = ctime
    cv2.putText(img, str(int(fps)), (30,70),cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 3)



    cv2.imshow("image", img)
    cv2.waitKey(1)