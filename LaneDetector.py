import cv2
import numpy as np
import time
from scipy.optimize import curve_fit
from Direction import Direction


IMAGE_SIZE = (1280, 720)

class Tracker:
    def __init__(self, position, width):
        self.position = [int(position[0]*IMAGE_SIZE[0]), int(position[1]*IMAGE_SIZE[1])]
        self.width = width
        self.x1 = self.position[0]-1 - self.width//2
        self.x2 = self.position[0]-1 + self.width//2
        self.y = self.position[1]-1
        self.pointer_x = self.position[0]
        self.is_active = True
        self.deviation = 0

    def draw(self, image):
        color = None
        if self.is_active:
            color = (0, 255, 0)
        else:
            color = (0, 0, 255)

        cv2.putText(image, self.get_direction().name + " " + str(round(self.deviation, 2)),
                    (self.position[0]-100, self.position[1]-20), 1, 2, (0, 0, 255), 2)
        
        cv2.line(image, (self.pointer_x, self.y-10), (self.pointer_x, self.y+10), color, 3)
        cv2.line(image, (self.x1, self.y), (self.x2, self.y), color, 3)
    
    def track(self, mask):
        center_of_lane_x = None
        lane_x1 = None
        lane_x2 = None

        for i in range(self.width+1):
            if (np.any(mask[self.y, self.x1:self.position[0]-1, 0] == 255) and \
                self.pointer_x <= self.position[0]) or \
                (np.any(mask[self.y, self.position[0]-1:self.x2, 0] == 255) and \
                self.pointer_x >= self.position[0]):
                self.is_active = True

            if self.is_active and mask[self.y, self.x1 + i, 0] == 255:
                lane_x1 = self.x1 + i
                lane_x2 = lane_x1
                i += 1
                while mask[self.y, self.x1 + i, 0] == 255 and i < self.width+1:
                    i += 1
                    lane_x2 += 1
                break

        if lane_x1 and lane_x2:
            center_of_lane_x = abs(lane_x2 + lane_x1)//2
            if center_of_lane_x < self.x1:
                center_of_lane_x = self.x1
            if center_of_lane_x > self.x2:
                center_of_lane_x = self.x2

        if center_of_lane_x:
            self.is_active = True
            self.pointer_x = center_of_lane_x

            self.deviation = (self.pointer_x-self.position[0])/(self.width//2)*100
        else:
            self.is_active = False

        return self.get_direction()

    def get_direction(self):
        if self.deviation < -20:
            return Direction.LEFT
        elif self.deviation > 20:
            return Direction.RIGHT
        return Direction.STRAIGHT

        


class LaneDetector:
    def __init__(self) -> None:
        self.distance_points = []
        self.center_pointer = [(IMAGE_SIZE[0]//2, IMAGE_SIZE[1]-30), (IMAGE_SIZE[0]//2, IMAGE_SIZE[1])]
        self.distances = []
        self.roi_height = 0.5

    def apply_mask(self, image):
        img = np.copy(image)
        hsv_transformed_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(hsv_transformed_frame, lower_red, upper_red)

        lower_red = np.array([150,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(hsv_transformed_frame, lower_red, upper_red)

        mask = mask0 + mask1
        mask = np.asarray(mask, np.uint8)
        # thresh = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        blur = cv2.GaussianBlur(mask,(25,25),0)
        thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        return thresh
    
    
    
    def get_roi(self, image):
        new_image = np.zeros(image.shape)
        start_y = int((1-self.roi_height)*IMAGE_SIZE[1])
        new_image[start_y:, :, :] = image[start_y:, :, :]
        return new_image
    
    def draw(self, image):
        cv2.line(image, self.center_pointer[0], self.center_pointer[1], (0, 0, 255), 3)

    def get_distance(self, mask, x=None):
        i = 0

        if not x:
            x = self.center_pointer[0][0]

        while i < self.center_pointer[1][1]+1:
            y = self.center_pointer[1][1]-1-i
            if mask[y, x, 0] == 255:
                return y
            i += 1
        return None

    def get_distances(self, mask):
        distance = self.get_distance(mask)
        if distance:
            self.distances.append(distance)
        else:
            self.distances = []

    def is_approaching_lane(self):
        if len(self.distances) > 3:
            if self.distances[0] < self.distances[-1]:
                return True
        return False
    
    def check_turn(self, mask):
        center_x = self.center_pointer[0][0]
        points = []
        for x in range(center_x - 50, center_x + 51, 10):
            distance = self.get_distance(mask, x)
            if distance is not None:
                points.append(distance)

        if len(points) > 2:
            popt, _ = curve_fit(lambda x, a, b: a * x + b, list(range(len(points))), points)
            a, _ = popt
            if a > 0:
                return Direction.LEFT
            else:
                return Direction.RIGHT
            
        return Direction.STRAIGHT
    
    def __call__(self, image):
        direction = None
        mask = self.apply_mask(image)
        mask = self.get_roi(mask)
        self.get_distances(mask)
        is_approaching_lane = self.is_approaching_lane()
        if is_approaching_lane:
            direction = self.check_turn(mask)
            cv2.putText(mask, direction.name + " turn detected", (100, 100), 1, 3, (0, 0, 255), 2)
        else:
            direction = Direction.STRAIGHT
            cv2.putText(mask, direction.name, (100, 100), 1, 3, (0, 0, 255), 2)

        self.draw(mask)

        return mask, direction
    