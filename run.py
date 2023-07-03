from sensors import Tracking, Obstacle, Ultrasonic, VideoStream
from loborobot import MotorRobot
import time
import threading
from queue import Queue
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_util import CVCar


class CyberTruck:
    motor = MotorRobot()
    manual_speed = 30  # speed
    manual_time = 0.1  # sec
    run_time = 0.0005  # sec, for autopilot

    def panel_click(self, key):
        eval(f"self.{key}()")

    def up(self):
        self.motor.t_up(self.manual_speed, self.manual_time)
        print("manual move forward")

    def back(self):
        self.motor.t_down(self.manual_speed, self.manual_time)
        print("manual move back")

    def left(self):
        self.motor.move_left(self.manual_speed, self.manual_time)
        print("manual move left")

    def right(self):
        self.motor.move_right(self.manual_speed, self.manual_time)
        print("manual move right")

    def leftup(self):
        self.motor.forward_left(self.manual_speed, self.manual_time)
        print("manual move forward left")

    def rightup(self):
        self.motor.forward_right(self.manual_speed, self.manual_time)
        print("manual move forward right")

    def speedup(self):
        self.manual_speed += 5
        print(f"Speed up! Now speed is: {self.manual_speed}")

    def speeddown(self):
        self.manual_speed -= 5
        print(f"Speed down! Now speed is: {self.manual_speed}")

    def carstop(self):
        #print("stop car here")
        self.motor.t_stop()

    def startcar(self):
        print("start car here")

    def testcar(self):
        print("test car here")

    def autoup(self):
        #print("autoup")
        self.motor.motor_run(0, 'forward', self.manual_speed+4.1)
        self.motor.motor_run(1, 'forward', self.manual_speed)
        self.motor.motor_run(2, 'forward', self.manual_speed+4)
        self.motor.motor_run(3, 'forward', self.manual_speed)
        time.sleep(self.run_time)

    def autodown(self):
        #print("autodown")
        self.motor.motor_run(0, 'backward', self.manual_speed)
        self.motor.motor_run(1, 'backward', self.manual_speed)
        self.motor.motor_run(2, 'backward', self.manual_speed)
        self.motor.motor_run(3, 'backward', self.manual_speed)
        time.sleep(self.run_time)

    def autoleft(self):
        #print("autoleft")
        self.motor.motor_run(0, 'backward', 20)
        self.motor.motor_run(1, 'forward', 30)
        self.motor.motor_run(2, 'backward', 20)
        self.motor.motor_run(3, 'forward', 30)
        time.sleep(self.run_time)

    def autoright(self):
        #print("autoright")
        self.motor.motor_run(0, 'forward', 30)
        self.motor.motor_run(1, 'backward', 20)
        self.motor.motor_run(2, 'forward', 30)
        self.motor.motor_run(3, 'backward', 20)
        time.sleep(self.run_time)

    def crossright(self):
        self.motor.motor_run(0, 'forward', self.manual_speed)
        self.motor.motor_run(1, 'backward', 20)
        self.motor.motor_run(2, 'forward', self.manual_speed)
        self.motor.motor_run(3, 'backward', 20)
        time.sleep(self.run_time)

    def autopilot(self):
        print(f"Cybertruck Autopilot start! The speed is {self.manual_speed}", time.time())
        on_cross = False
        last_run = "autoup"
        outcnt = 0
        status = "start"
        sensor = SensorDetection()
        cross_list = ["up", "up", "right", "right", "left", "left"]
        cross_i = 0
        cross_marker = 0
        stop_sign = False
        #time.sleep(5)
        for i in range(1000000000000):
            if not cv_to_pilot.empty():
                cv_msg = cv_to_pilot.get()
                #print(f"message received: {cv_msg}")
                if cv_msg == "stop":
                    #print("cv stop detected!")
                    self.carstop()
                    stop_sign = True
                    continue
                elif cv_msg == "go":
                    #print("cv go detected!")
                    stop_sign = False
                    pass
            if stop_sign:
                self.carstop()
                continue
            msg = sensor.decision()
            #if msg != status:
                #print(f"{status} -> {msg}")
            if msg == "unknown":
                eval(f"self.{last_run}()")
            elif msg == "onroute":
                self.autoup()
                last_run = "autoup"
            elif msg == "outofroute":
                #print("OUT OF ROUTE! BREAK!")
                outcnt += 1
                print("outcnt", outcnt)
                if outcnt > 50:
                    break
                else:
                    eval(f"self.{last_run}()")
            elif msg == "leftshift":
                self.autoright()
                last_run = "autoright"
            elif msg == "rightshift":
                self.autoleft()
                last_run = "autoleft"
            elif msg == "oncross":
                print("on_cross:", i)
                if i - cross_marker < 30:
                    eval(f"self.{last_run}()")
                else:
                    if cross_i >= len(cross_list):
                        break
                    turn = cross_list[cross_i]
                    print(f"cross {cross_i}: {turn}")
                    if turn == "up":
                        print("cross up here")
                        for _ in range(20):
                            self.autoup()
                        if last_run == "autoleft":
                            last_run = "autoright"
                        elif last_run == "autoright":
                            last_run = "autoleft"
                    else:
                        for _ in range(80):
                            eval(f"self.auto{turn}()")
                        last_run = f"auto{turn}"
                    cross_i += 1
                    cross_marker = i
            else:
                self.autoup()
            status = msg
        pilot_to_cv.put("car_stop")
        self.carstop()


class SensorDetection:
    freq = 0.005  # detect frequency
    cycle = 20  # detect cycle number
    on_route = True  # whether on route?
    left = False  # whether left shift?
    right = False  # whether right shift?
    tracking = Tracking()
    obs = Obstacle()
    ultrasonic = Ultrasonic()

    def sensor_read(self, tracking=False, infrared=False, ultrasonic=False):
        res = []
        if tracking:
            tracking_left_value,tracking_right_value,tracking_center_value  = self.tracking.tracking_detect()  # read tracking sensor
            res.append([tracking_left_value,tracking_right_value,tracking_center_value])
        if infrared:
            infra_left_value,infra_right_value = self.obs.infra_detect()  # read infrared sensor
            res.append([infra_left_value,infra_right_value])
        if ultrasonic:
            distance = self.ultrasonic_distance()  # read ultrasonic sensor
            res.append(distance)
        return res

    def get_data(self):
        tracking = []
        obs = []
        for _ in range(self.cycle):
            sensor = self.sensor_read(tracking=True, infrared=True)
            tracking.append(sensor[0])
            obs.append(sensor[1])
            #time.sleep(self.freq)
        tracking = np.array(tracking)
        obs = np.array(obs)
        return tracking, obs

    def decision(self):
        tracking, obs = self.get_data()
        left = tracking[:, 0].sum()*2 // self.cycle
        center = tracking[:, 1].sum()*2 // self.cycle
        right = tracking[:, 2].sum()*2 // self.cycle
        # tracking decision
        if left + center + right == 0:  # out of route, need adjust to the route
            #print("OUT of Route!")
            res = "outofroute"
        elif center >= 1 and left + right == 0:
            #print("ON Route!")
            res = "onroute"
        elif left >= 1 and right == 0:  # shift to the right
            #print("RIGHT Shift!")
            res = "rightshift"
        elif right >= 1 and left == 0:  # shift to the left
            #print("LEFT Shift!")
            res = "leftshift"
        elif (right == 2 and center == 2) or (left == 2 and center == 2):
            #print("On Cross")
            res = "oncross"
        else:
            res = "unknown"
        return res

    def run(self):
        is_break = False
        while not is_break:
            self.decision()
            while not pilot_to_sensor.empty():
                is_break = True
            time.sleep(0.01)

    def plot(self):
        print(time.time())
        tracking, obs = self.get_data()
        print(time.time())
        plt.plot(tracking[:, 0], "o-", label="left")
        plt.plot(tracking[:, 1], "^-", label="center")
        plt.plot(tracking[:, 2], "*-", label="right")
        plt.legend()
        plt.show()


class CVDetection:
    video = VideoStream()
    #model = YOLO("v0.0_yolov8n.pt")

    def get_data(self):
        frame = self.video.get_frame(0)
        return frame

    def predict(self, image):
        model = CVCar(image)
        result = model.detect()
        if result in ["red", "yellow"]:
            return "stop"
        else:
            return "go"

    def decision(self):
        pred = self.predict(image)
        if "left" in pred.keys():
            return "left"
        elif "right" in pred.keys():
            return "right"
        elif "stop" in pred.keys():
            return "stop"
        elif "green" in pred.keys():
            return "go"
        elif "red" in pred.keys():
            return "stop"
        elif "yellow" in pred.keys():
            return "stop"

    def run(self):
        while True:
            if not pilot_to_cv.empty():
                msg = pilot_to_cv.get()
                if msg == "car_stop":
                    print("Car stopped, CV inference end!")
                    break
            image = self.get_data()
            #print("cv predict start:", time.time())
            result = self.predict(image)
            cv_to_pilot.put(result)
            #print("cv predict end:", time.time())            
            cv2.imwrite(f"images/{int(time.time()*1000000)}.jpg", image)
            #cv2.imshow('RTSP Stream', frame)
            #cv2.waitKey(1)
            #print("image saved")
            time.sleep(0.05)


def car_run():
    cybertruck = CyberTruck()
    cybertruck.autopilot()
    cybertruck.carstop()


def cv_detection():
    detection = CVDetection()
    detection.run()


def cv_simulate():
    l = ["go"] * 10 + ["stop"] * 5
    for _ in range(1000):
        for ll in l:
            cv_to_pilot.put(ll)
            time.sleep(0.1)


if __name__ == "__main__":
    cv_to_pilot = Queue()
    pilot_to_cv = Queue()
    # 创建线程
    t1 = threading.Thread(target=car_run)
    t2 = threading.Thread(target=cv_detection)
    # 启动线程
    t1.start() 
    t2.start()
