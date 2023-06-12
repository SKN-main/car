
from LaneDetector import LaneDetector, Tracker
from RoadSignsDetector import RoadSignsDetector
from Direction import Direction
import cv2
import time
from Queue import Queue
from collections import Counter
from copy import deepcopy


class Car:
    def __init__(self) -> None:
        self.lane_detector = LaneDetector()
        self.left_tracker = Tracker((300, 400), 190)
        self.right_tracker = Tracker((950, 400), 190)
        self.road_signs_detector = RoadSignsDetector('20e_street_20e_printed.pt')
        self.direction = Direction.STRAIGHT
        self.turn = None

        self.possible_signs = Queue(8)
        self.possible_directions = Queue(10)
        self.prev_sign = None
        self.detected_sign = None


    def detect_road_sign(self, image):
        nearest_sign = None

        self.road_signs_detector.predict(image)
        signs = self.road_signs_detector.get_predicted_signs()

        nearest_signs = sorted(signs, key=lambda t: abs(t[1]-t[3])*abs(t[0]-t[2]))

        if len(nearest_signs):
            nearest_sign = nearest_signs[0][4]

        self.possible_signs.add(nearest_sign)

        most_common_sign = Counter(self.possible_signs.array).most_common()[0][0]


        if most_common_sign == 'c12':
            self.prev_sign = deepcopy(self.detected_sign)
            self.detected_sign = 'c12'
        elif most_common_sign == 'b20':
            self.prev_sign = deepcopy(self.detected_sign)
            self.detected_sign = 'b20'
        elif most_common_sign == 'c2':
            self.prev_sign = deepcopy(self.detected_sign)
            self.detected_sign = 'c2'
        elif most_common_sign == 'c4':
            self.prev_sign = deepcopy(self.detected_sign)
            self.detected_sign = 'c4'
        else:
            self.detected_sign = None

    
    def __call__(self, frame):
        self.detect_road_sign(frame)

        mask, turn = self.lane_detector(frame)

        self.turn = turn
        
        left_tracker_direction = self.left_tracker.track(mask)
        right_tracker_direction = self.right_tracker.track(mask)


        if self.turn.value == Direction.STRAIGHT.value:
            if left_tracker_direction.value == Direction.STRAIGHT.value:
                self.direction = left_tracker_direction
            elif right_tracker_direction.value == Direction.STRAIGHT.value:
                self.direction = right_tracker_direction
            else:
                self.direction = right_tracker_direction
        else:
            if self.turn.value == Direction.LEFT.value:
                if self.left_tracker.is_active:
                    self.direction = right_tracker_direction
                    self.turn = Direction.STRAIGHT
                else:
                    self.direction = self.turn
            elif self.turn.value == Direction.RIGHT.value:
                if self.right_tracker.is_active:
                    self.direction = left_tracker_direction
                    self.turn = Direction.STRAIGHT
                else:
                    self.direction = self.turn


        if self.detected_sign == None:
            if self.prev_sign == 'c12':
                self.direction == Direction.RIGHT
                if right_tracker_direction.value == Direction.LEFT.value and self.right_tracker.is_active and left_tracker_direction.value == Direction.LEFT.value:
                    self.prev_sign = None
            if self.prev_sign == 'b20':
                # Stop and wait 5 seconds
                pass

            # if self.prev_sign ==


        cv2.putText(mask, self.direction.name, (500 ,500), 1, 3, (0, 0, 255), 3)
        if self.turn.value != Direction.STRAIGHT.value:
            cv2.putText(mask, "Turn: " + self.turn.name, (500 ,550), 1, 3, (0, 0, 255), 3)
        if self.detected_sign:
            cv2.putText(frame, "sign: " + self.detected_sign, (500 ,600), 1, 3, (0, 0, 255), 3)
        if self.prev_sign:
            cv2.putText(frame, "prev sign: " + self.prev_sign, (500 ,650), 1, 3, (0, 0, 255), 3)


        self.left_tracker.draw(mask)
        self.right_tracker.draw(mask)

        return mask, self.direction

def show_image():
    image = cv2.imread('image.png')
    mask = car(image)
    cv2.imshow('Frame', mask)
    cv2.waitKey()

def show_video():
    is_finish = False
    while not is_finish:
        is_finish = False
        cap = cv2.VideoCapture('przykladowa_trasa.mp4')
        # cap = cv2.VideoCapture('Untitled.mp4')
        success, image = cap.read()
        while success and not is_finish:
            success, image = cap.read()
            if not success:
                break
            
            
            mask, direction = car(image)


            cv2.imshow('Mask', mask)
            cv2.imshow('Frame', image)


            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                is_finish = 1
            if key == ord('s'):
                if cv2.waitKey() & 0xFF == ord('q'):
                    is_finish = 1

        time.sleep(0.1) 


if __name__ == '__main__':
    car = Car()
    show_video()


