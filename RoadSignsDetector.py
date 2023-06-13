import torch
import cv2
import numpy as np


class RoadSignsDetector():
    CLASS_NAMES = ['c4', 'c2', 'b20', 'c12']
    COLORS = [(255, 0, 0), (0, 255, 255), (0, 255, 0), (0, 0, 255)]

    def __init__(self, weights, confidence_rate=0.5) -> None:
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights)
        self.predicted_signs = []
        self.confidence_rate = confidence_rate

    def predict(self, frame):
        # original_size = frame.shape[:2][::-1]
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # img = cv2.resize(img, (640, 640))

        img_to_pred = np.zeros(frame.shape)
        img_to_pred[:frame.shape[0]//2, :, :] = img[:frame.shape[0]//2, :, :]

        result = self.model(img_to_pred)


        # rendered_img = result.render()[0]
        # rendered_img = cv2.resize(rendered_img, original_size)
        # rendered_img = cv2.cvtColor(rendered_img, cv2.COLOR_RGB2BGR)

        self.predicted_signs = []
        for pred in result.xyxyn[0]:
            if self.confidence_rate > float(pred[4]):
                continue

            class_name = self.CLASS_NAMES[int(pred[5])]
            self.predicted_signs.append((*pred[:4].numpy(), class_name))
        
        for x1, y1, x2, y2, class_name in self.predicted_signs:
            x1 = int(x1*img.shape[1])
            y1 = int(y1*img.shape[0])
            x2 = int(x2*img.shape[1])
            y2 = int(y2*img.shape[0])
            color = self.COLORS[self.CLASS_NAMES.index(class_name)]
            img = cv2.rectangle(img, (x1, y1), (x2, y2), color, 5)
            img = cv2.putText(img,  class_name, (x1, y1), 1, 3, color, 5)

        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img
    
    def get_predicted_signs(self):
        """
        The function returns a list of detected road signs as: (x1, y1, x2, y2, class_name).
        The coordintates are represented in the form of floats which are relative distances from left top corner of the screen.
        """
        return self.predicted_signs

if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    detector = RoadSignsDetector('weights/20e_street_20e_printed.pt')

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        img = detector.predict(frame)
        predicted_signs = detector.get_predicted_signs()

        if len(predicted_signs):
            cv2.putText(img, f"Znak {predicted_signs[0][-1]}", (50, 75), 1, 5, (0, 0, 255), 2)

        cv2.imshow('frame', img)
        if cv2.waitKey(1) == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()

