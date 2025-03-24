import cv2
import mediapipe as mp
import time
import math



class pose_estim():
    def __init__(self, mode = False, upBody = False, smooth = True, detectconfid = 0.5, trackconfid = 0.5):
        self.mode = mode
        self.upBody = upBody
        self.smooth = smooth
        self.detectconfid = detectconfid
        self.trackconfid = trackconfid
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(
            static_image_mode=self.mode,
            #upper_body_only=self.upBody,
            smooth_landmarks=self.smooth,
            min_detection_confidence=self.detectconfid,
            min_tracking_confidence=self.trackconfid
        )
        self.mpDraw = mp.solutions.drawing_utils

    def findpose(self, img,draw = True):
        imageRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imageRGB)
        # print(results.pose_landmarks)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks,
                                      self.mpPose.POSE_CONNECTIONS)  # mediapipe website :- 32 points of body

        return img

    def getposition(self,img, draw = True):
        self.lmlist = []
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                #print(id, lm)  # the vales are in decimals are just the ratio of the image
                cx, cy = int(lm.x * w), int(lm.y * h) # gives the x point of pixel value
                self.lmlist.append([id,cx,cy])
                if draw:
                   cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        return self.lmlist
    def getangle(self, img, id1, id2, id3, draw= True):
        #getting landmmarks
        #_, x1,y1 = self.lmlist[id1]
        #consider [3,245,383] is coordinates of id = 3 and x,y values. In the first line we slice the coordinate
        # removing the id 3 and getting only x and values. [1:] states give value 1 till end (245 and 383). Can also use the second comented method too
        x1, y1 = self.lmlist[id1][1:]
        x2, y2 = self.lmlist[id2][1:]
        x3, y3 = self.lmlist[id3][1:]
        #calculate angle
        angle = math.degrees(math.atan2(y3-y2,x3-x2) - math.atan2(y1-y2,x1-x2))

        if angle < 0:
            angle += 360
        #print(angle)
        if draw:
            cv2.line(img, (x1,y1),(x2,y2), (255,0,255),3)
            cv2.line(img, (x3, y3), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, (x1, y1), 5, (255, 255, 0), cv2.FILLED)
            cv2.circle(img, (x1, y1), 15, (255, 255, 0), 2)
            cv2.circle(img, (x2, y2), 5, (255, 255, 0), cv2.FILLED)
            cv2.circle(img, (x3, y3), 5, (255, 255, 0), cv2.FILLED)
            cv2.putText(img,str(int(angle)), (x2-20,y2+40),cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2 )
        return angle
def main():
    ptime = 0  # prevtime
    ctime = 0  # current time
    #cap = cv2.VideoCapture('pose_videos/3.mp4')  # webcam number
    cap = cv2.VideoCapture(0)#enter video name over here eg 'video.mp4'
    detect = pose_estim()


    while True:
        success, img = cap.read()
        img = detect.findpose(img)
        lmlist = detect.getposition(img)
        # if len(lmlist) != 0:
        #     print(lmlist[6])
        #cv2.circle(img, (lmlist[14][1], lmlist[14][2]), 5,(0, 0, 255), cv2.FILLED)

        ctime = time.time()
        fps = 1 / (ctime - ptime)
        ptime = ctime
        cv2.putText(img, str(int(fps)), (10, 10), cv2.FONT_HERSHEY_DUPLEX, 3, (255, 0, 255), 3)
        cv2.imshow("image", img)
        cv2.waitKey(1)  # delay

if __name__ == "__main__":
    main()