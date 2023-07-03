import cv2
import numpy as np

name = ["green", "red"]
path = "img//traffic_lights_"

for filename in name:
    img = cv2.imread(path + str(filename) + ".jpg")
    
    hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    cv2.imshow("hsv_" + str(filename), v)
    
    red = np.sum(v[50:150, 20:120])
    yellow = np.sum(v[175:275, 20:120])
    green = np.sum(v[290:390, 20:120])
    
    print(str(red) + "|" + str(yellow) + "|" + str(green))
    if red >= yellow and red >= green:
        print("Red")
    if yellow >= red and yellow >= green:
        print("Yellow")
    if green >= yellow and green >= red:
        print("Green")
    else:
        print("Error")

class CVCar:
    lower_red = np.array([0, 120, 70])  # red
    upper_red = np.array([10, 255, 255])
    lower_yellow = np.array([20, 100, 100])  # yellow
    upper_yellow = np.array([40, 255, 255])
    lower_green = np.array([40, 40, 40])  # green
    upper_green = np.array([70, 255, 255])
    lower_blue = np.array([90, 50, 50])  # blue
    upper_blue = np.array([135, 255, 255])

    def __init__(self, image):
        self.image = image

    def detect(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        # mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        edges_red = cv2.Canny(mask_red, 100, 200)
        edges_yellow = cv2.Canny(mask_yellow, 100, 200)
        edges_green = cv2.Canny(mask_green, 100, 200)
        # edges_blue = cv2.Canny(mask_blue, 100, 200)
        self.circle_detect(edges_red)  # traffic light red
        self.circle_detect(edges_yellow)  # traffic light yellow
        self.circle_detect(edges_green)  # traffic light green
        # self.ellipse_detect(edges_blue)  # turn sign blue

    def circle_detect(self, edges):
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, contour in enumerate(contours):
            if contour.shape[0] > 100:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                contour_area = cv2.contourArea(contour)
                circle_area = np.pi * radius * radius
                print(i, contour.shape, contour_area, circle_area)
                if circle_area != 0:
                    print(contour_area / circle_area)
                # if circle_area != 0 and contour_area/circle_area > 0.6:
                cv2.drawContours(self.image, contour, -1, (0, 255, 0), 2)
                cv2.circle(self.image, center, radius, (0, 0, 255), 2)

    def ellipse_detect(self, edges):
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, contour in enumerate(contours):
            if contour.shape[0] > 20:
                ellipse = cv2.fitEllipse(contour)
                center, axes, angle = ellipse
                major_axis, minor_axis = axes

                contour_area = cv2.contourArea(contour)
                circle_area = np.pi * major_axis * minor_axis
                print(i, contour.shape, contour_area, circle_area)
                if circle_area != 0:
                    print(contour_area / circle_area)
                # if circle_area != 0 and contour_area/circle_area > 0.6:
                cv2.drawContours(self.image, contour, -1, (0, 255, 0), 2)
                cv2.ellipse(self.image, ellipse, (0, 0, 255), 2)


if __name__ == "__main__":
    from sensors import VideoStream
    video = VideoStream()

