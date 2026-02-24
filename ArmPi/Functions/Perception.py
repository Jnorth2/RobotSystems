#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
import atexit


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# __target_color = ('red',)
# # 设置检测颜色
# # Set detection color
# def setTargetColor(target_color):
#     global __target_color

#     #print("COLOR", target_color)
#     __target_color = target_color
#     return (True, ())

# # 找出面积最大的轮廓
# # 参数为要比较的轮廓的列表
# # Find the contour with the largest area
# # Parameter is a list of contours to compare
# def getAreaMaxContour(contours):
#     contour_area_temp = 0
#     contour_area_max = 0
#     area_max_contour = None

#     for c in contours:  # 历遍所有轮廓 Go through all contours
#         contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积Calculate contour area
#         if contour_area_temp > contour_area_max:
#             contour_area_max = contour_area_temp
#             if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰 The maximum area contour is only valid when the area is greater than 300 to filter out interference
#                 area_max_contour = c

#     return area_max_contour, contour_area_max  # 返回最大的轮廓 Return the largest contour

# # 夹持器夹取时闭合的角度 The angle at which the gripper closes when clamping
# servo1 = 500

# # 初始位置 initial position
# def initMove():
#     Board.setBusServoPulse(1, servo1 - 50, 300)
#     Board.setBusServoPulse(2, 500, 500)
#     AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

# def setBuzzer(timer):
#     Board.setBuzzer(0)
#     Board.setBuzzer(1)
#     time.sleep(timer)
#     Board.setBuzzer(0)

# #设置扩展板的RGB灯颜色使其跟要追踪的颜色一致 Set the RGB light color of the expansion board to match the color you want to track
# def set_rgb(color):
#     if color == "red":
#         Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
#         Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
#         Board.RGB.show()
#     elif color == "green":
#         Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
#         Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
#         Board.RGB.show()
#     elif color == "blue":
#         Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
#         Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
#         Board.RGB.show()
#     else:
#         Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
#         Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
#         Board.RGB.show()

# count = 0
# track = False
# _stop = False
# get_roi = False
# center_list = []
# first_move = True
# __isRunning = False
# detect_color = 'None'
# action_finish = True
# start_pick_up = False
# start_count_t1 = True
# # 变量重置 variable reset
# def reset():
#     global count
#     global track
#     global _stop
#     global get_roi
#     global first_move
#     global center_list
#     global __isRunning
#     global detect_color
#     global action_finish
#     global start_pick_up
#     global __target_color
#     global start_count_t1
    
#     count = 0
#     _stop = False
#     track = False
#     get_roi = False
#     center_list = []
#     first_move = True
#     __target_color = ()
#     detect_color = 'None'
#     action_finish = True
#     start_pick_up = False
#     start_count_t1 = True

# # app初始化调用 app initialization call
# def init():
#     print("ColorTracking Init")
#     initMove()

# # app开始玩法调用 app starts gameplay call
# def start():
#     global __isRunning
#     reset()
#     __isRunning = True
#     print("ColorTracking Start")

# # app停止玩法调用 app stops gameplay call
# def stop():
#     global _stop 
#     global __isRunning
#     _stop = True
#     __isRunning = False
#     print("ColorTracking Stop")

# # app退出玩法调用 app exit gameplay call
# def exit():
#     global _stop
#     global __isRunning
#     _stop = True
#     __isRunning = False
#     print("ColorTracking Exit")

# rect = None
# size = (640, 480)
# rotation_angle = 0
# unreachable = False
# world_X, world_Y = 0, 0
# world_x, world_y = 0, 0
# # 机械臂移动线程 Robotic arm moving thread
# def move():
#     global rect
#     global track
#     global _stop
#     global get_roi
#     global unreachable
#     global __isRunning
#     global detect_color
#     global action_finish
#     global rotation_angle
#     global world_X, world_Y
#     global world_x, world_y
#     global center_list, count
#     global start_pick_up, first_move

#     # 不同颜色木快放置坐标(x, y, z) Quick placement coordinates of different colors of wood
#     coordinate = {
#         'red':   (-15 + 0.5, 12 - 0.5, 1.5),
#         'green': (-15 + 0.5, 6 - 0.5,  1.5),
#         'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
#     }
#     while True:
#         if __isRunning:
#             if first_move and start_pick_up: # 当首次检测到物体时 When an object is first detected               
#                 action_finish = False
#                 set_rgb(detect_color)
#                 setBuzzer(0.1)               
#                 result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0) # 不填运行时间参数，自适应运行时间 If the running time parameter is not filled in, the running time will be adaptive.
#                 if result == False:
#                     unreachable = True
#                 else:
#                     unreachable = False
#                 time.sleep(result[2]/1000) # 返回参数的第三项为时间 The third item of the return parameter is time
#                 start_pick_up = False
#                 first_move = False
#                 action_finish = True
#             elif not first_move and not unreachable: # 不是第一次检测到物体 This is not the first time an object is detected
#                 set_rgb(detect_color)
#                 if track: # 如果是跟踪阶段 If it is the tracking stage
#                     if not __isRunning: # 停止以及退出标志位检测 Stop and exit flag detection
#                         continue
#                     AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
#                     time.sleep(0.02)                    
#                     track = False
#                 if start_pick_up: #如果物体没有移动一段时间，开始夹取 If the object has not moved for a while, start gripping
#                     action_finish = False
#                     #Claw and roll adjustment
#                     if not __isRunning: # 停止以及退出标志位检测 Stop and exit flag detection
#                         continue
#                     Board.setBusServoPulse(1, servo1 - 280, 500)  # 爪子张开 Claws spread
#                     # 计算夹持器需要旋转的角度 Calculate the angle by which the gripper needs to be rotated
#                     servo2_angle = getAngle(world_X, world_Y, rotation_angle)
#                     Board.setBusServoPulse(2, servo2_angle, 500)
#                     time.sleep(0.8)
                    
#                     #Move down
#                     if not __isRunning:
#                         continue
#                     AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # 降低高度 lower height
#                     time.sleep(2)
                    
#                     #Close gripper
#                     if not __isRunning:
#                         continue
#                     Board.setBusServoPulse(1, servo1, 500)  # 夹持器闭合 Gripper closed
#                     time.sleep(1)
                    
#                     #Lift up
#                     if not __isRunning:
#                         continue
#                     Board.setBusServoPulse(2, 500, 500)
#                     AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起 Robotic arm raised
#                     time.sleep(1)
                    
#                     #Place in Bin
#                     if not __isRunning:
#                         continue
#                     # 对不同颜色方块进行分类放置 Classify and place blocks of different colors
#                     result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
#                     time.sleep(result[2]/1000)
                    
#                     if not __isRunning:
#                         continue
#                     servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
#                     Board.setBusServoPulse(2, servo2_angle, 500)
#                     time.sleep(0.5)

#                     if not __isRunning:
#                         continue
#                     AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
#                     time.sleep(0.5)
                    
#                     if not __isRunning:
#                         continue
#                     AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
#                     time.sleep(0.8)
                    
#                     if not __isRunning:
#                         continue
#                     Board.setBusServoPulse(1, servo1 - 200, 500)  # 爪子张开，放下物体 Open your claws and drop the object
#                     time.sleep(0.8)
                    
#                     if not __isRunning:
#                         continue                    
#                     AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
#                     time.sleep(0.8)

#                     initMove()  # 回到初始位置 Return to initial position
#                     time.sleep(1.5)

#                     detect_color = 'None'
#                     first_move = True
#                     get_roi = False
#                     action_finish = True
#                     start_pick_up = False
#                     set_rgb(detect_color)
#                 else:
#                     time.sleep(0.01)
#         else:
#             if _stop:
#                 _stop = False
#                 Board.setBusServoPulse(1, servo1 - 70, 300)
#                 time.sleep(0.5)
#                 Board.setBusServoPulse(2, 500, 500)
#                 AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
#                 time.sleep(1.5)
#             time.sleep(0.01)

# # 运行子线程 Run child thread
# th = threading.Thread(target=move)
# th.setDaemon(True)
# th.start()

# t1 = 0
# roi = ()
# last_x, last_y = 0, 0
# def run(img):
#     global roi
#     global rect
#     global count
#     global track
#     global get_roi
#     global center_list
#     global __isRunning
#     global unreachable
#     global detect_color
#     global action_finish
#     global rotation_angle
#     global last_x, last_y
#     global world_X, world_Y
#     global world_x, world_y
#     global start_count_t1, t1
#     global start_pick_up, first_move
    
#     img_copy = img.copy()
#     img_h, img_w = img.shape[:2]
#     cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
#     cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
    
#     if not __isRunning:
#         return img
     
#     frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
#     frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
#     #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止 If a recognized object is detected in a certain area, the area is detected until there is no recognized object.
#     if get_roi and start_pick_up:
#         get_roi = False
#         frame_gb = getMaskROI(frame_gb, roi, size)    
    
#     frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间 Convert image to LAB space
    
#     area_max = 0
#     areaMaxContour = 0
#     if not start_pick_up:
#         for i in color_range:
#             if i in __target_color:
#                 detect_color = i
#                 frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # 对原图像和掩模进行位运算 Perform bit operations on the original image and mask
#                 opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算 Open operation
#                 closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算 closed operation
#                 contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓 find the outline
#                 areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓 Find the maximum contour
#         if area_max > 2500:  # 有找到最大面积 Found the largest area
#             rect = cv2.minAreaRect(areaMaxContour)
#             box = np.int0(cv2.boxPoints(rect))

#             roi = getROI(box) #获取roi区域 Get roi area
#             get_roi = True

#             img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标 Get the center coordinates of the wooden block
#             world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标 Convert to real world coordinates
            
            
#             cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
#             cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) #绘制中心点 draw center point
#             distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动 Compare the last coordinates to determine whether to move
#             last_x, last_y = world_x, world_y
#             track = True
#             #print(count,distance)
#             # 累计判断 Cumulative judgment
#             if action_finish:
#                 if distance < 0.3:
#                     center_list.extend((world_x, world_y))
#                     count += 1
#                     if start_count_t1:
#                         start_count_t1 = False
#                         t1 = time.time()
#                     if time.time() - t1 > 1.5:
#                         rotation_angle = rect[2]
#                         start_count_t1 = True
#                         world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
#                         count = 0
#                         center_list = []
#                         start_pick_up = True
#                 else:
#                     t1 = time.time()
#                     start_count_t1 = True
#                     count = 0
#                     center_list = []
#     return img

class ArmPerception():
    def __init__(self, colors, size):
        self.camera = Camera.Camera()
        self.camera.camera_open()
        self.frame = None
        self.get_roi = False
        self.start_pick = False
        self.color_range = {}
        self.last_color = None
        self.last_center = [0,0]
        self.current_center = [0,0]
        self.center_list = []
        self.rotation_angle = None
        self.avg_center = [0,0]
        self.start_pick_up = False
        self.seen_count = 0
        self.t1 = time.time()
        self.start_count_t1 = True
        self.count = 0
        self.roi = None
        self.size = size
        self.make_color_range(colors)

        atexit.register(self.stop)
    
    def get_frame(self):
        frame = self.camera.frame
        self.frame = frame.copy()

    def stop(self):
        self.camera.camera_close()
        cv2.destroyAllWindows()

    def make_color_range(self, colors):
        for i in colors:
            self.color_range[i] = color_range[i]

    def preprocess(self, frame):
        frame_resize = cv2.resize(frame, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        if self.get_roi and not self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    
    
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB) 

        return frame_lab
    
    def get_largest_contour(self, frame):
        if self.last_color is not None:
            contour, max_A = self.get_contour(frame, self.color_range[self.last_color])
            if max_A > 2500:
                self.seen_count +=1
                return contour, max_A, self.last_color
        max_contour, max_A = 0   
        color = None
        for key, value in self.color_range:
            temp_contour, temp_A = self.get_contour(frame, value)
            if temp_contour is not None:
                    if temp_A > max_A:#找最大面积
                        max_A = temp_A
                        color = key
                        max_contour = temp_contour
        if max_A > 2500:
            self.seen_count = 0
            self.last_color = color
            return max_contour, max_A, color
        else:
            self.last_color = None
            return None, None, None
    
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # 历遍所有轮廓 Go through all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰 The maximum area contour is only valid when the area is greater than 300 to filter out interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # 返回最大的轮廓 Return the largest contour


    def get_contour(self, frame, color):
        frame_mask = cv2.inRange(frame, color[0], color[1])  # 对原图像和掩模进行位运算 Perform bit operations on the original image and mask
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算 Open operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算 closed operation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓 find the outline
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓 Find the maximum contour
        return areaMaxContour, area_max
    
    def make_roi(self, contour):
        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))
        
        self.roi = getROI(box) #获取roi区域
        self.get_roi = True
        img_centerx, img_centery = getCenter(rect, self.roi, self.size, square_length)  # 获取木块中心坐标
        
        self.current_center[0], self.current_center[1] = convertCoordinate(img_centerx, img_centery, self.size) #转换为现实世界坐标
        return rect, box
    
    def track_target(self, rect):
        distance = math.sqrt(pow(self.current_center[0] - self.current_center[1], 2) + pow(self.current_center[1] - self.last_center[0], 2)) #对比上次坐标来判断是否移动 Compare the last coordinates to determine whether to move
        self.last_center = self.current_center
        #print(count,distance)
        # 累计判断 Cumulative judgment
        if distance < 0.5:
            self.count += 1
            self.center_list.extend((self.current_center[0], self.current_center[1]))
            if self.start_count_t1:
                self.start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1:
                self.rotation_angle = rect[2] 
                self.start_count_t1 = True
                self.avg_center[0], self.avg_center[1] = np.mean(np.array(self.center_list).reshape(count, 2), axis=0)
                self.center_list = []
                self.count = 0
                if self.seen_count == 3:
                    self.start_pick_up = True

        else:
            self.t1 = time.time()
            self.start_count_t1 = True
            self.center_list = []
            self.count = 0
        if self.seen_count == 3:
            self.seen_count = 0
            self.draw_color = range_rgb[self.last_color]
        else:
            self.draw_color = range_rgb['black']

    def draw_image(self, img, box = None, max_A = None):

        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if box is not None and max_A is not None:
            cv2.drawContours(img, [box], -1, range_rgb[max_A], 2)
            cv2.putText(img, '(' + str(self.current_center[0]) + ',' + str(self.current_center[1]) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[max_A], 1) #绘制中心点
            
        cv2.putText(img, "Color: " + self.last_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.last_color, 2)

        return img
    
    def camera_loop(self):
        self.get_frame()
        frame = self.preprocess(self.frame.copy())
        max_contour, max_A, color = self.get_largest_contour(frame)
        box = None
        if max_contour is not None:
            rect, box = self.make_roi(max_contour)
            self.track_target(rect)
        img = self.draw_image(self.frame, box, max_A)


    
        

if __name__ == '__main__':
    # init()
    # start()
    # __target_color = ('red', )
    # my_camera = Camera.Camera()
    # my_camera.camera_open()
    # while True:
    #     img = my_camera.frame
    #     if img is not None:
    #         frame = img.copy()
    #         Frame = run(frame)           
    #         cv2.imshow('Frame', Frame)
    #         key = cv2.waitKey(1)
    #         if key == 27:
    #             break
    # my_camera.camera_close()
    # cv2.destroyAllWindows()

    perception = ArmPerception(colors="red",size=(640, 480))
    while True:
        img = perception.camera_loop()
        cv2.imshow('Frame', img)
        key = cv2.waitKey(1)
        if key == 27:
            break