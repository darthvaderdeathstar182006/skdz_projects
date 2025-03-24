import cv2
import mediapipe as mp
import time

class handetect():
    def __init__(self,mode=False,maxhandz =2, detectconfident= 0.5, trackconfident=0.5):
        self.mode = mode
        self.maxhandz = maxhandz
        self.detection = detectconfident
        self.track = trackconfident
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
            min_detection_confidence=self.detection,
            min_tracking_confidence=self.track,
            max_num_hands=self.maxhandz
        )
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, img, draw= True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)  # processes the frame and gives result
        # print(results.multi_hand_landmarks)#gives the coordinates of one hand(x,y,z) and none for no hands visible

        if self.results.multi_hand_landmarks:
           for handLms in self.results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(img, handLms)#draws small point on 21 coordinates of hands
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,self.mpHands.HAND_CONNECTIONS)  # CONNECTS ALL 21 COORDINATES OF THE HAND
        return img

    def findpositionpoint(self, img, handno = 0, draw = True):
        lmlist = []
        #idlist = []
        if self.results.multi_hand_landmarks:
            myhand = self.results.multi_hand_landmarks[handno]
            for idno, lm in enumerate(myhand.landmark):

                #print(id,lm)  # prints id(number of the respective points of hand) and their coordinates gives img ratio
                h, w, c = img.shape
                cx, cy , cz = int(lm.x * w), int(lm.y * h), int(lm.z)  # multiply by height and width to get pixel value
                #print(id, cx, cy)
                lmlist.append([idno, cx, cy])
                # if idno == id:
                if draw:
                   cv2.circle(img, (cx, cy), 7, (255, 0, 255),cv2.FILLED)  # draws a circle at the id 0 of the hand
        return lmlist
    def drawcircle(self, img, idno, handno = 0):
        if self.results.multi_hand_landmarks:
            myhand = self.results.multi_hand_landmarks[handno]
            for id, lm in enumerate(myhand.landmark):
                h, w, c = img.shape
                cx, cy, cz = int(lm.x * w), int(lm.y * h), int(lm.z)  # multiply by height and width to get pixel value
                if id == idno:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 255), cv2.FILLED)
        return img

#def main is a dummy code use this part and check for whats missing in diff project to access these things
def main():
    ptime = 0  # prevtime
    ctime = 0  # current time
    cap = cv2.VideoCapture(0)  # webcam number
    detect = handetect()

    while True:
        success, img = cap.read()
        img = detect.findHands(img)
        lmlist = detect.findpositionpoint(img)
        if len(lmlist) != 0:
            print(lmlist[4])
        # calculate frame rate
        ctime = time.time()
        fps = 1 / (ctime - ptime)
        ptime = ctime
        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_DUPLEX, 3, (255, 0, 255), 3)
        # position


        cv2.imshow("image", img)
        cv2.waitKey(1)  # delay


if __name__ == "__main__":
    main()




















 #(ctrl 0/ to comment selected lines)


