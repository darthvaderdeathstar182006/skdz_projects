import mediapipe as mp
import cv2
import time


class facemeshdetector():
    def __init__(self,
               static_image_mode=False,
               max_num_faces=2,
               refine_landmarks=False,
               min_detection_confidence=0.5,
               min_tracking_confidence=0.5):
        self.static_image_mode = static_image_mode
        self.max_num_faces = max_num_faces
        self.refine_landmarks = refine_landmarks
        self.min_tracking_confidence = min_tracking_confidence
        self.min_detection_confidence = min_detection_confidence
        self.mpDraw = mp.solutions.drawing_utils
        self.mpFaceMesh = mp.solutions.face_mesh
        self.facemesh = self.mpFaceMesh.FaceMesh(self.static_image_mode,self.max_num_faces,self.refine_landmarks,self.min_tracking_confidence,self.min_detection_confidence)
        self.drawspec = self.mpDraw.DrawingSpec(thickness=1, circle_radius=2)

    def findfacemesh(self, img, idno, point = True, draw=True):
        self.imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.facemesh.process(self.imgRGB)
        faces = []
        if self.results.multi_face_landmarks:
            for facelms in self.results.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, facelms, self.mpFaceMesh.FACEMESH_CONTOURS, self.drawspec,
                                               self.drawspec)
                if point:
                    face = []  # thickness   #circleradius
                    for facelms in self.results.multi_face_landmarks:
                        for id, lm in enumerate(facelms.landmark):
                            # print(lm)#gives xyz coordinates of the respective point
                            ih, iw, ic = img.shape
                            x, y = int(lm.x * iw), int(lm.y * ih)
                            # cv2.putText(img, str(id), (x, y), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)#print repective id numbers instead of points on your face
                            # print(0, x, y)  # id / name of respective point(0 till 467)
                            face.append([x, y])  # good for only one face
                            if id == idno:
                                cv2.circle(img, (x, y), 7, (255, 0, 255),
                                           cv2.FILLED)  # draws a circle at the id of the face point
                        faces.append(face)  # helps for multiple face
            return img, faces
def main():
    cap = cv2.VideoCapture(0)
    ptime = 0
    detector = facemeshdetector()
    while True:
        success, img = cap.read()
        img, faces = detector.findfacemesh(img, 0, True)
        #if len(faces) != 0:
        print(len(faces))
        #print(faces)#print all points

        ctime = time.time()
        fps = 1 / (ctime - ptime)
        ptime = ctime
        cv2.putText(img, str(int(fps)), (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
        cv2.imshow("image", img)
        cv2.waitKey(1)










if __name__ == "__main__":
    main()