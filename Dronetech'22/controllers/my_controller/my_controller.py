###################################################FINAL#######################################


from controller import Robot, Motor, Gyro, GPS, Camera, Compass, Keyboard, LED, InertialUnit, DistanceSensor
import math
import cv2
import numpy as np
from PIL import Image
from pyzbar.pyzbar import decode
import math


SIGN = lambda x: int(x>0) - int(x<0)
CLAMP = lambda value, low, high : min(high, max(value, low))


class Drone:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.camera_front = self.robot.getDevice('camera_front');
        self.camera_front.enable(self.timestep)
        self.camera_down = self.robot.getDevice('camera'); # importing downward facing camera
        self.camera_down.enable(self.timestep)
        # front_left_led = robot.getDevice("front left led");
        # front_right_led = robot.getDevice("front right led");
        self.imu = self.robot.getDevice("inertial unit");
        self.imu.enable(self.timestep);
        self.gps = self.robot.getDevice("gps");
        self.gps.enable(self.timestep);
        self.compass = self.robot.getDevice("compass");
        self.compass.enable(self.timestep);
        self.gyro = self.robot.getDevice("gyro");
        self.gyro.enable(self.timestep);
        self.ds_front=self.robot.getDevice("ds_front")
        self.ds_front.enable(self.timestep)
        self.ds_right=self.robot.getDevice("ds_right")
        self.ds_right.enable(self.timestep)
        self.ds_left=self.robot.getDevice("ds_left")
        self.ds_left.enable(self.timestep)
       
        # keyboard = Keyboard();
        # keyboard.enable(timestep)
        self.camera_roll_motor = self.robot.getDevice('camera roll');
        self.camera_pitch_motor = self.robot.getDevice('camera pitch');

        self.front_left_motor = self.robot.getDevice("front left propeller");
        self.front_right_motor = self.robot.getDevice("front right propeller");
        self.rear_left_motor = self.robot.getDevice("rear left propeller");
        self.rear_right_motor = self.robot.getDevice("rear right propeller");
        self.motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor];

        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        self.k_vertical_thrust = 68.5
        self.k_vertical_offset = 0.6 
        self.k_vertical_p = 3.0
        self.k_roll_p = 50.0
        self.k_pitch_p = 30.0

        self.target_altitude = 1.0
        self.prev_error = 0
        self.integral = 0
        self.target_x = 21.8
        self.target_y = 0.772
        self.prev_x = 0
        self.prev_y = 0
        self.integral_x = 0
        self.integral_y = 0

    def move(self,command,intensity, intensity2=0):
        roll = self.imu.getRollPitchYaw()[0] + math.pi / 2.0
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        
        self.roll=roll
        self.pitch=pitch
        self.yaw=yaw
        altitude = self.gps.getValues()[1]
        roll_acceleration = self.gyro.getValues()[0]
        pitch_acceleration = self.gyro.getValues()[1]
        yaw_accelration = self.gyro.getValues()[2]
        x = self.gps.getValues()[0]
        y = self.gps.getValues()[2]
        self.x=x
        self.y=y
        self.altitude=altitude


        # led_state = int(time) % 2
        # front_left_led.set(led_state)
        # front_right_led.set(int(not led_state))
        
        self.camera_roll_motor.setPosition(-0.115 * roll_acceleration)
        self.camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)
        
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        if(command=='forward'):
            self.target_x -= intensity  #2.0
        elif(command=='backward'):
            self.target_x += intensity #-2.0
        elif(command=='right'):
            yaw_disturbance = intensity  #1.3
        elif(command=='left'):
            yaw_disturbance = -intensity  #-1.3
        elif(command=='sRight'):
            self.target_y -= intensity  #-1.0
        elif(command=='sLeft'):
            self.target_y += intensity  #1.0
        elif(command=='up'):
            self.target_altitude += intensity  #0.05
        elif(command=='down'):
            self.target_altitude -= intensity  #0.05
        elif(command=="beech_mei"):
            self.target_x -= intensity
            self.target_y += intensity2

        P = 10      
        I = 0.05
        D =  220
        
        P_X = 2      
        I_X = .0001
        D_X =  500
        
        P_Y = 10      
        I_Y = 0.05
        D_Y =  220
        motion_x = -(self.target_x - x)
        motion_y = (self.target_y - y)
        if abs(motion_x) < 0.1:
            self.integral_x += motion_x
        else:
            self.integral_x = 0
        if abs(motion_y) < 0.1:
            self.integral_y += motion_y
        else:
            self.integral_y = 0
        pid_x = P_X*motion_x + D_X*(motion_x - self.prev_x) + I_X*self.integral_x
        self.prev_x = motion_x
        pid_y = P_Y*motion_y + D_Y*(motion_y - self.prev_y) + I_Y*self.integral_y
        self.prev_y = motion_y


        pitch_disturbance = CLAMP(pid_x, -0.5, 0.5)
        roll_disturbance = CLAMP(pid_y, -0.5, 0.5)
        error = self.target_altitude - altitude
        derivative = error - self.prev_error
        if abs(error) < 0.1:
            self.integral += error
        else:
            self.integral = 0
        pid_hieght = P * error + D * derivative + I * self.integral
        self.prev_error = error
        roll_input = self.k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = self.k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance + yaw_accelration
        clamped_difference_altitude = CLAMP(pid_hieght, -1.0, 1.0)
        vertical_input = self.k_vertical_p * pow(clamped_difference_altitude, 3.0)

        # print(motion_x, motion_y)
        front_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        front_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
        rear_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        rear_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        self.front_left_motor.setVelocity(front_left_motor_input)
        self.front_right_motor.setVelocity(-front_right_motor_input)
        self.rear_left_motor.setVelocity(-rear_left_motor_input)
        self.rear_right_motor.setVelocity(rear_right_motor_input)
        
    def get_image_down(self):
        self.camera_down.saveImage('image'+'.jpg', 100)
        image = cv2.imread('image'+'.jpg')
        return image
        
    def get_image_front(self):
        self.camera_front.saveImage('image_front.jpg', 100)
        image = cv2.imread('image_front.jpg')
        return image

def blue(image):
    img=image
    
    lower = np.array([110,50,50])
    upper = np.array([130,255,255])
    l=lower
    u=upper
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, l, u)

    contours, hierarchy = cv2.findContours(mask,
                                       cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
    k=0
    for contour in contours:
    
        area = cv2.contourArea(contour) 
        if (area>15000):  
            print("area:",area)
            k=k+1
    print("k:",k)
    if(k==0):
      print("not found")
      return 0
    else:
      print("found") 
      return 1
       
def green(image):
    img=image
    
    lower = np.array([25, 52, 72])
    upper = np.array([102, 255,255])
    l=lower
    u=upper
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, l, u)

    contours, hierarchy = cv2.findContours(mask,
                                       cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
    k=0
    for contour in contours:
        area = cv2.contourArea(contour)     
        if (3000<area<19000):  
            print("area:",area)  
            print("green")
            k=k+1

    if(k==0):
      print("not found")
      return 0
    else:
      print("found")
      return 1

def red(image):
    img=image
    
    lower = np.array([0,50,70])
    upper = np.array([9,255,255])
    l=lower
    u=upper
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, l, u)

    contours, hierarchy = cv2.findContours(mask,
                                       cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)
    k=0
    print(k)
    for contour in contours:
        area = cv2.contourArea(contour)
        if (area>5000):  
            k=k+1
    
    print("k:",k)      

    if(k==0):
      print("not found")
      return 0
    else:
      print("found")
      return 1

def decode_the_data(data):
    if(data!=None):
        shape = (data.split(")")[0]).split("(")[1]
        rgb= ((data.split(")")[1]).split("(")[1]).split(",")
        rgb[0], rgb[1], rgb[2] =int(rgb[0]), int(rgb[1]), int(rgb[2])
        return shape, rgb     

def find_max_index(x):
    max_index = 0
    for i in range(len(x)):
        if x[i] > x[max_index]:
            max_index = i
    return max_index

        
def qr_code(image):
    img = image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    Data=None
    for bqcode in decode(img):
        Data = bqcode.data.decode('utf-8')
        pts = np.array([bqcode.polygon], np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(img, [pts], True, (0, 255, 0), 2)
        pts2 = bqcode.rect
        cv2.putText(img, Data, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
        if(Data==None):
            # print("no qr code detected")
            pass
        else:
            print(Data)
            print(type(Data))
    return Data

def shape(img,low,high):
  hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv,low,high)
  cont, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  max_cnt = max(cont,key = cv2.contourArea())
  peri = cv2.arclength(max_cnt)
  shape = cv2.approxPolyDP(max_cnt,0.04*peri,True)
  if len(shape) == 4:
    return "Square"
  else:
    return "Cylinder"




drone=Drone()

shape=""
rgb=[]
i=0
j=0
qr=0
k=0
dat=None
while drone.robot.step(drone.timestep) != -1:
    i=i+1
    if(i==1):
        drone.move('up',4-1.5)
    else:       
        drone.move('forward', 0)
    if(4.8-1.5<drone.altitude<5.04-1.5 and qr==0):
        if(j<10):
            drone.move("forward", .5)
            j=j+1
            
        diff=drone.x-drone.target_x
        if(diff<2):
            img = drone.get_image_down()
            qr_code(img)
            if(qr_code(img)!=None):
                dat=qr_code(img)
                shape,rgb=decode_the_data(dat)
                

                qr=1
    elif(qr==1):
        print("changed")
        m=0
        while(m<10):
            drone.move('backward', 0)
            m=m+1
        drone.target_x=drone.x - .5
        k=1
    if(dat!=None and k==1):
        k=0
        qr=2
        drone.target_x=0
    if(abs(drone.target_x-drone.x)<.2 and qr==2):
        drone.target_y=0
        while drone.robot.step(drone.timestep) != -1:
            drone.move('right', 0)
            if(abs(drone.target_y-drone.y)<.2):
                break
        break

while drone.robot.step(drone.timestep) != -1:
    drone.move('left', .5)
    print("leftttttttttttttttttt")
    img=drone.get_image_front()
    max_index=find_max_index(rgb)
    
    # red()
    if(max_index==0):
        val=red(img)
        if (val==1):
            img2=drone.get_image_front() 
            print("found red color")
            print("stopppppppppppppppppppppppppppppppppppppppppppppppppp")
            break
    # green()
    elif(max_index==1):
        val=green(img)
        if (val==1):
            print("found green color")
            print("stopppppppppppppppppppppppppppppppppppppppppppppppppp")
            break
    # blue()  
    else:
        val=blue(img)
        if (val==1):
            img2=drone.get_image_front()
            print("found blue color")
            print("stopppppppppppppppppppppppppppppppppppppppppppppppppp")
            break
    print("Mission Achieved")


for i in range(0,100):
    print("yaw: "+str(drone.yaw))
 
error=math.pi+drone.yaw 
tan_theta= error 

x_m=5
y_m=x_m*tan_theta
print(tan_theta)
print(x_m, y_m)


i=0
while drone.robot.step(drone.timestep) != -1:
    print("x loop")
    if(i==0):
        drone.move('forward', x_m)
        i=1
    drone.move('forward', 0)
    if(abs(drone.x-drone.target_x)<.2):
        print("X-done")
        while drone.robot.step(drone.timestep) != -1:
            print("y-loop")
            if(i==1):
                drone.move('left', y_m)
                i=2
            drone.move('left', 0)
            print(drone.y-drone.target_y)
            if(abs(drone.y-drone.target_y)<.01):
                print("Y-done")
                i=0
                break