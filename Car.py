
from LaneDetector import LaneDetector, Tracker
from RoadSignsDetector import RoadSignsDetector
from Direction import Direction
import cv2
import time
from Queue import Queue
from collections import Counter
from copy import deepcopy

IMAGE_SIZE = (1280, 720)


class Car:
    def __init__(self) -> None:
        self.lane_detector = LaneDetector()
        self.left_tracker = Tracker((0.2, 0.7), 200)
        # self.right_tracker = Tracker((0.8, 0.7), 100)
        self.right_tracker = Tracker((0.8, 0.7), 200)
        # self.left_tracker = Tracker((250, 500), 190)
        # self.right_tracker = Tracker((950, 500), 190)
        self.direction = Direction.STRAIGHT
        self.turn = None
        self.recent_tracker = self.left_tracker
        self.sign_direction = None


        self.road_signs_detector = RoadSignsDetector('20e_street_20e_printed.pt')
        self.possible_signs = Queue(8)
        self.possible_directions = Queue(10)
        self.prev_sign = None
        self.detected_sign = None

    def detect_road_sign(self, image):
        nearest_sign = None

        image = self.road_signs_detector.predict(image)
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
        
        return image

    def straight(self):
        left_tracker_dir_val =  self.left_tracker.get_direction().value
        right_tracker_dir_val =  self.right_tracker.get_direction().value

        if Direction.STRAIGHT.value in [left_tracker_dir_val, right_tracker_dir_val]:
            self.direction = Direction.STRAIGHT

        if self.recent_tracker is not None:
            self.direction = self.recent_tracker.get_direction()

        if left_tracker_dir_val == right_tracker_dir_val:
            self.direction = self.left_tracker.get_direction() 
    
    def __call__(self, frame):
        frame_with_road_sign = self.detect_road_sign(frame)
        mask, turn = self.lane_detector(frame)

        self.left_tracker.track(mask)
        self.right_tracker.track(mask)

        # choose tracker

        if self.right_tracker.is_active:
            self.recent_tracker = self.right_tracker
        elif self.left_tracker.is_active:
            self.recent_tracker = self.left_tracker

        if self.turn is not None:
            if self.turn.value == Direction.LEFT and not self.right_tracker.is_active:
                self.direction = Direction.LEFT
                self.turn = Direction.LEFT
            elif self.turn.value == Direction.RIGHT and not self.left_tracker.is_active:
                self.direction = Direction.RIGHT
                self.turn = Direction.RIGHT
        else:
            if turn.value == Direction.LEFT.value:
                self.direction = Direction.LEFT
                self.recent_tracker = self.right_tracker
            elif turn.value == Direction.RIGHT.value:
                self.direction = Direction.RIGHT
                self.recent_tracker = self.left_tracker
            else:
                self.straight()
                self.turn = None


        if (self.prev_sign == 'c12' or self.prev_sign == 'c2') and self.detected_sign is None:
            self.sign_direction = Direction.RIGHT
            if turn is not None or turn.value != Direction.STRAIGHT.value:
                self.direction = deepcopy(self.sign_direction)
                if self.right_tracker.is_active and self.right_tracker.get_direction().value == Direction.LEFT.value:
                    self.prev_sign = None
                    self.sign_direction = None

        if self.prev_sign == 'c4' and self.detected_sign is None:
            self.sign_direction = Direction.LEFT
            if turn is not None or turn.value != Direction.STRAIGHT.value:
                self.direction = deepcopy(self.sign_direction)
                if self.left_tracker.is_active and self.left_tracker.get_direction().value == Direction.RIGHT.value:
                    self.prev_sign = None
                    self.sign_direction = None

        if self.prev_sign == 'b20' and self.detected_sign is None:
            pass
            # stop the car



        cv2.putText(mask, self.direction.name, (500 ,500), 1, 3, (0, 0, 255), 3)
        if self.turn is not None:
            cv2.putText(mask, "Turn: " + self.turn.name, (500 ,550), 1, 3, (0, 0, 255), 3)
        if self.detected_sign:
            cv2.putText(frame_with_road_sign, "sign: " + self.detected_sign, (500 ,600), 1, 3, (0, 0, 255), 3)
        if self.prev_sign:
            cv2.putText(frame_with_road_sign, "prev sign: " + self.prev_sign, (500 ,650), 1, 3, (0, 0, 255), 3)


        self.left_tracker.draw(mask)
        self.right_tracker.draw(mask)

        cv2.imshow("frame", frame_with_road_sign)

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
        cap = cv2.VideoCapture(0)
        # cap = cv2.VideoCapture('przykladowa_trasa.mp4')
        # cap = cv2.VideoCapture('Untitled.mp4')
        success, image = cap.read()
        while success and not is_finish:
            success, image = cap.read()
            if not success:
                break
            
            image = cv2.resize(image, IMAGE_SIZE)
            mask, direction = car(image)


            # cv2.imshow('image', image)
            cv2.imshow('Frame', mask)

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


