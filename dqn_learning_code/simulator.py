#!/usr/bin/env python3
import cv2
import numpy as np
import yaml, sys, time, random
# import rospy, roslib, rospkg

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Vector3
# from tt_core_msgs.msg import Vector3DArray, ImagePoint
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import CompressedImage

simulator = {"width":31, "height":31, "center":15, "resol":2} # Size of the windows (Just for visualization)
map_param = {"width":50, "height":50, "center":25, "resol":1, "scale":5} # Size of the map (Should be same with real one?)
## simulator resol could be non interge to make precise in our case. our robots 304*502
## it is more nature when we use simulator resol is 2 then 10cm per 1 simulator then .. pixel 3*5.. 
## 5cm per 1 map pixel ## when simulator resol is 3, 15cm per 1 simulator pixel 
# number of pixels behind
non_detect = 7
Back_pixels = 6
margin = 6 ## MARGIN 
ball_margin = 8
Ran_wall = 20 # random lengthen wall size (늘어난 벽의 길이)
camera_fov = 78
obstacle_base = 2/5*map_param["height"] ## 
obstacle_length = int(2/5*map_param["width"]) ##
ball_blind_ratio = 1/np.tan(camera_fov/2*np.pi/180)
ball_blind_bias = 1

reward_region_x = [-1,0,1,-2,2]
reward_region_y = 3
trans_scale = int(simulator["resol"]/map_param["resol"])
rot_scale = 20

debug_scale = 10
debug_scale_gray = 3

max_iter = 149 # why only 99?

# We also want to train the sorting plate so that it can move properly when it detect the color of the ball
sorting_plate_state_dic = {'NULL': 0, 'RED': 1, 'BLUE': 2}

class Task:
    def __init__(self, debug_flag=False, test_flag=False, state_blink=True, state_inaccurate=True):
        # This is in simulator scale
        self.frame = np.zeros((simulator["height"],simulator["width"],1), np.uint8)
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray,simulator["width"]*debug_scale_gray,1), np.uint8)
        # This is in map scale
        self.red_balls = []
        self.blue_balls = []
        self.red_balls_prev = []
        self.blue_balls_prev = []
        self.obstacles = []
        self.state_dis_cor = []
        self.state_dis_cen = []
        self.robots_cen = [] #center
        self.robots_cor = [] #right above cor
        # For ...
        self.episode_rewards = []
        self.current_reward = 0
        self.score = 0
        self.iter = 0
        self.sorting_plate_state = sorting_plate_state_dic['RED']
        self.num_state_after_pick = 0
        self.done = False
        # For ...
        self.write_flag = False
        self.debug_flag = debug_flag
        self.test_flag = test_flag
        self.ball_inscreen_flag = 0
        self.state_blink = state_blink
        self.state_inaccurate = state_inaccurate
        # DQN parameters
        self.observation_space = self.frame_gray.copy()
        self.action_space = np.array(range(7))
        # ROS
        # rospack = rospkg.RosPack()
        # root = rospack.get_path('tt_rl_motion_planner')
        # path = root+"/config/map_gen.yaml"
        path = "./map_gen.yaml"
        stream = open(path, 'r')
        self._params = yaml.load(stream)
        return

    def reset(self, max_balls=10, max_walls=2):
        self.frame = np.zeros((simulator["height"], simulator["width"], 1), np.uint8)
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray, simulator["width"]*debug_scale_gray, 1), np.uint8)
        self.red_balls = []
        self.blue_balls = []
        self.red_balls_prev = []
        self.blue_balls_prev = []
        self.robots = []
        self.state_dis_cor = []
        self.state_dis_cen = []
        self.obstacles = []
        self.robots_cen = []  # center
        self.robots_cor = []  # right above cor
        self.current_reward = 0
        self.score = 0
        self.iter = 0
        self.sorting_plate_state = sorting_plate_state_dic['RED']
        self.num_state_after_pick = 0
        self.done = False
        self.write_flag = False
        self.ball_inscreen_flag = 0

        # 비디오 녹화 주기를 줄여주었다.
        if len(self.episode_rewards)%200 == 0 and not self.test_flag:
            self.write_flag = True
            out_directory = "data/video/tt.video."+format(int(len(self.episode_rewards)/200),"08")+".mp4"

        if self.test_flag:
            self.write_flag = True
            out_directory = "data/video_test/tt.video."+format(int(len(self.episode_rewards)),"08")+".mp4"

        if self.write_flag:
            codec = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 10
            self.video = cv2.VideoWriter(out_directory, codec, fps, (simulator["width"]*debug_scale,simulator["height"]*debug_scale))

        #           ^
        #           | y
        # +-------------------+
        # |    \    |         |
        # |     \i_t|         |
        # |      \  |         | w_h
        # |       \ |         |
        # |        \|         |
        # |         +---------|-----> x
        # |         |         |
        # |         |         |
        # |         |         |
        # |         |         |
        # |         |         |
        # +-------------------+
        #          w_w

        # Rotate everything to robot's frame
        #   [x'] = [ cos   sin][x - r_x]
        #   [y']   [-sin   cos][y - r_y]
        
        # Create environment (In map scale)
        walls_initial = []
        obstacles_initial = []
        obstacles_temp = []
        i_t = random.random()*np.pi - np.pi/2 # initial random theta, the angle difference with map orientation and car orientation.
        ran_obs = random.random()
        rand_map = random.random()
        if rand_map <= 0.333:
            # map without walls nor obstacles
            w_w = map_param["width"]
            w_h = map_param["height"]
            r_x = (random.random()-0.5)*(w_w - 2*margin) # initial robot rx map base
            r_y = -(w_h-2*margin)/2 + random.random()*(obstacle_base-2*margin) # initial robot ry map base
            walls_initial = []
        else:
            if rand_map >= 0.666:
                # map with walls only
                w_w = map_param["width"] + round(random.random()*Ran_wall) # random wall width
                w_h = map_param["height"] + round(random.random()*Ran_wall) # random wall height
                r_x = (random.random()-0.5)*(w_w - 2*margin) # initial robot rx map base
                r_y = -(w_h-2*margin)/2 + random.random()*(obstacle_base-2*margin) # initial robot ry map base
            else:
                # map with walls and obstacles
                w_w = map_param["width"] + round(random.random()*Ran_wall)
                w_h = map_param["height"] + round(random.random()*Ran_wall) + round(obstacle_base)
                r_x = (random.random()-0.5)*(w_w - 2*margin) # initial robot rx map base
                r_y = -(w_h-2*margin)/2 + random.random()*(obstacle_base-2*margin) # initial robot ry map base
                for i in range(obstacle_length):
                    ox = (-w_w/2) + ran_obs*(w_w - obstacle_length) + i ##obstacle's x coordinate
                    obstacles_initial.append([ox, -w_h/2 + obstacle_base]) 
                    for obstacle in obstacles_initial:
                        x = obstacle[0]
                        y = obstacle[1]
                        t_x = np.cos(i_t)*(x - r_x) + np.sin(i_t)*(y - r_y)
                        t_y = -np.sin(i_t)*(x - r_x) + np.cos(i_t)*(y - r_y)
                        obstacles_temp.append([t_x, t_y])
            for i in range(w_w):
                cx = -round(w_w/2) + i
                cy = -round(w_h/2)
                walls_initial.append([cx, cy])
            for i in range(w_h):
                cx = -round(w_w/2) + w_w
                cy = -round(w_h/2) + i
                walls_initial.append([cx, cy])
            for i in range(w_w):
                cx = -round(w_w/2) + w_w - i
                cy = -round(w_h/2) + w_h
                walls_initial.append([cx, cy])
            for i in range(w_h):
                cx = -round(w_w/2)
                cy = -round(w_h/2) + w_h - i
                walls_initial.append([cx, cy])
        m_x = np.cos(i_t)*(0 - r_x) + np.sin(i_t)*(0 - r_y)
        m_y = -np.sin(i_t)*(0 - r_x) + np.cos(i_t)*(0 - r_y)
        mc_x = np.cos(i_t)*(w_w/2 - r_x) + np.sin(i_t)*(w_h/2 - r_y)
        mc_y = -np.sin(i_t)*(w_w/2 - r_x) + np.cos(i_t)*(w_h/2 - r_y)
        self.robots_cen=[[m_x,m_y]]
        self.robots_cor=[[mc_x,mc_y]]
        # Rotate everything to robot's frame
        #   [x'] = [ cos   sin][x - r_x]
        #   [y']   [-sin   cos][y - r_y]
        for wall in walls_initial:
            x = wall[0]
            y = wall[1]
            f_x = np.cos(i_t)*(x - r_x) + np.sin(i_t)*(y - r_y)
            f_y = -np.sin(i_t)*(x - r_x) + np.cos(i_t)*(y - r_y)
            obstacles_temp.append([f_x, f_y])

        for obstacle in obstacles_temp:
            cx = obstacle[0]
            cy = obstacle[1]
            self.obstacles.append([cx, cy])
        ##################### End of the Map constructing ###################

        # Place balls randomly
        for i in range(max_balls):
            cx = int(1.0*(2*random.random() - 1)*(w_w/2 - margin)) ## ball x-cor 
            cy = int(-w_h/2 + obstacle_base + margin + random.random()*(w_h - obstacle_base - 2*margin)) ##ball y-cor
            f_x = np.cos(i_t)*(cx - r_x) + np.sin(i_t)*(cy - r_y)
            f_y = -np.sin(i_t)*(cx - r_x) + np.cos(i_t)*(cy - r_y)
            insert = True
            for b in self.red_balls:
                if (b[0]-f_x)*(b[0]-f_x) + (b[1]-f_y)*(b[1]-f_y) < (ball_margin)*(ball_margin): ##ball margin
                    insert = False
                    break
            for b in self.blue_balls:
                if (b[0]-f_x)*(b[0]-f_x) + (b[1]-f_y)*(b[1]-f_y) < (ball_margin)*(ball_margin):
                    insert = False
                    break
            if insert:
                if i < max_balls/2:
                    self.red_balls.append([f_x,f_y])
                else:
                    self.blue_balls.append([f_x,f_y])
        self.draw_state()

        return self.frame_gray

    def check_window_map(self, cx, cy):
        inscreen = True
        if cx < 0 or cx >= map_param["width"]:
            inscreen = False
        if cy < 0 or cy >= map_param["height"]:
            inscreen = False
        return inscreen

    def check_window_state(self, cx, cy):
        inscreen = True
        if cx < 0 or cx >= simulator["width"]:
            inscreen = False
        if cy < 0 or cy >= simulator["height"]:
            inscreen = False
        return inscreen

    def draw_debug_frame(self, frame):
        frame_debug = np.zeros((simulator["height"]*debug_scale,simulator["width"]*debug_scale,3), np.uint8)

        # For reference

        # cv::rectangle
        # img       Image.
        # pt1       Vertex of the rectangle.
        # pt2       Vertex of the rectangle opposite to pt1 .
        # color     Rectangle color or brightness (grayscale image).
        # thickness Thickness of lines that make up the rectangle.
        #           Negative values, like FILLED, mean that the function has to draw a filled rectangle.

        # cv::line
        # img       Image.
        # pt1       First point of the line segment.
        # pt2       Second point of the line segment.
        # color     Line color.
        # thickness Line thickness.
        # lineType  Type of the line. See LineTypes.
        # shift     Number of fractional bits in the point coordinates.
        
        # Draw the grid
        for i in range(1, simulator["width"]):
            # Vertical grid
            cv2.line(frame_debug,
                     (i*debug_scale, 0),
                     (i*debug_scale, simulator["height"]*debug_scale - 1),
                     (50, 50, 50),
                     1)
            # Horizontal grid
            cv2.line(frame_debug,
                     (0, i*debug_scale),
                     (simulator["width"]*debug_scale - 1, i*debug_scale),
                     (50, 50, 50),
                     1)
        
        # Draw the obstacles
        for i in range(simulator["width"]):
            for j in range(simulator["height"]):
                if frame[i][j] == self._params["Map.data.obstacle"]:
                    cv2.rectangle(frame_debug,
                                  (i*debug_scale, j*debug_scale),
                                  ((i + 1)*debug_scale - 1, (j + 1)*debug_scale - 1),
                                  (255, 255, 0),
                                  -1)
                if frame[i][j] == self._params["Map.data.red_ball"]:
                    cv2.rectangle(frame_debug,
                                  (i*debug_scale, j*debug_scale),
                                  ((i + 1)*debug_scale - 1, (j + 1)*debug_scale - 1),
                                  (0, 0, 255),
                                  -1)
                if frame[i][j] == self._params["Map.data.blue_ball"]:
                    cv2.rectangle(frame_debug,
                                  (i*debug_scale, j*debug_scale),
                                  ((i + 1)*debug_scale - 1, (j + 1)*debug_scale - 1),
                                  (255, 0, 0),
                                  -1)
        
        # Center of the robot
        cv2.rectangle(frame_debug,
                      (simulator["center"]*debug_scale, (simulator["height"] - Back_pixels)*debug_scale + 1),
                      ((simulator["center"] + 1)*debug_scale, (simulator["height"] - (Back_pixels - 1))*debug_scale - 1),
                      (255, 0, 0) if self.sorting_plate_state == sorting_plate_state_dic['BLUE'] else (0, 0, 255),
                      -1)

        # Boundary of the robot
        cv2.rectangle(frame_debug,
                      ((simulator["center"] - 2)*debug_scale - 1,
                       (simulator["height"] - (Back_pixels + 2))*debug_scale + 1),
                      ((simulator["center"] + 3)*debug_scale,
                       (simulator["height"] - (Back_pixels - 3))*debug_scale - 1),
                      (0, 0, 255),
                      2)
        # roller of the robot
        cv2.rectangle(frame_debug,
                      ((simulator["center"] - 1)*debug_scale - 1,
                       (simulator["height"] - (Back_pixels + 2))*debug_scale + 1),
                      ((simulator["center"] + 2)*debug_scale,
                       (simulator["height"] - (Back_pixels + 1))*debug_scale - 1),
                      (0, 255, 0),
                      2)

        # Right line view angle
        cv2.line(frame_debug,
                 ((simulator["center"] + ball_blind_bias)*debug_scale, (simulator["height"] - Back_pixels)*debug_scale - 1),
                 (simulator["width"]*debug_scale - 1, ((simulator["height"] - Back_pixels) - int(ball_blind_ratio*(simulator["center"] - 1 - ball_blind_bias)))*debug_scale),
                 (128, 128, 128),
                 1)
        # Left line view angle
        cv2.line(frame_debug,
                 ((simulator["center"] - ball_blind_bias + 1)*debug_scale, (simulator["height"] - Back_pixels)*debug_scale - 1),
                 (0, (simulator["height"] - Back_pixels - int(ball_blind_ratio*(simulator["center"] - 1 - ball_blind_bias)))*debug_scale),
                 (128, 128, 128),
                 1)
        # Text
        # cv2.putText(frame_debug,
        #             "Score " + str(self.score),
        #             (int(simulator["width"]*debug_scale*0.65), int(simulator["width"]*debug_scale*0.05)),
        #             cv2.FONT_HERSHEY_TRIPLEX,
        #             0.05*debug_scale,
        #             (255, 255, 255))
        # cv2.putText(frame_debug,
        #             "Step " + str(self.iter),
        #             (int(simulator["width"]*debug_scale*0.05),  int(simulator["width"]*debug_scale*0.05)),
        #             cv2.FONT_HERSHEY_TRIPLEX,
        #             0.5,
        #             (255, 255, 255))

        # Text (New)
        cv2.putText(frame_debug,
                    "Step " + str(self.iter),
                    (int(simulator["width"]*debug_scale*0.70), int(simulator["width"]*debug_scale*0.05)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*debug_scale,
                    (255, 255, 255))
        cv2.putText(frame_debug,
                    "Score " + '{:.3f}'.format(self.score),
                    (int(simulator["width"]*debug_scale*0.05), int(simulator["width"]*debug_scale*0.05)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*debug_scale,
                    (255, 255, 255))
        cv2.putText(frame_debug,
                    "Reward " + str(self.current_reward),
                    (int(simulator["width"]*debug_scale*0.05),  int(simulator["width"]*debug_scale*0.10)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*debug_scale,
                    (255, 255, 255))

        return frame_debug

    def draw_state_gray(self):
        #gray_color = {"red_ball":255, "blue_ball":220, "wall":100, "robot":200, "robot_padding":150}
        # 다음 교육에는 sorting plate 상태에 따라 gray에도 표시해줌
        gray_color = {"red_ball":255, "blue_ball":50, "wall":100, "robot_red":200, "robot_blue":125, "robot_padding":150,  "robot_roller":175}
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray,simulator["width"]*debug_scale_gray,1), np.uint8)
        ran_gray_x=int(random.random()*debug_scale_gray)-1 
        ran_gray_y=int(random.random()*debug_scale_gray)-1
        ran_gray_x =0 
        ran_gray_y = 0
        for i in range(simulator["width"]):
            for j in range(simulator["height"]):
                if self.frame[i][j] == self._params["Map.data.obstacle"]:
                    cv2.rectangle(self.frame_gray,
                                  (i*debug_scale_gray+ran_gray_x, j*debug_scale_gray+ran_gray_y),
                                  ((i + 1)*debug_scale_gray - 1+ran_gray_x, (j + 1)*debug_scale_gray - 1+ran_gray_y),
                                  gray_color["wall"],
                                  -1)
                if self.frame[i][j] == self._params["Map.data.red_ball"]:
                    cv2.rectangle(self.frame_gray,
                                  (i*debug_scale_gray+ran_gray_x, j*debug_scale_gray+ran_gray_y),
                                  ((i + 1)*debug_scale_gray - 1+ran_gray_x, (j + 1)*debug_scale_gray - 1+ran_gray_y),
                                  gray_color["red_ball"],
                                  -1)
                if self.frame[i][j] == self._params["Map.data.blue_ball"]:
                    cv2.rectangle(self.frame_gray,
                                  (i*debug_scale_gray+ran_gray_x, j*debug_scale_gray+ran_gray_y),
                                  ((i + 1)*debug_scale_gray - 1+ran_gray_x, (j + 1)*debug_scale_gray - 1+ran_gray_y),
                                  gray_color["blue_ball"],
                                  -1)

        ### gray color of the bots
        cv2.rectangle(self.frame_gray,
                      ((simulator["center"] - 2)*debug_scale_gray+ran_gray_x, (simulator["height"] - (Back_pixels + 2))*debug_scale_gray + +ran_gray_y),
                      ((simulator["center"] + 3)*debug_scale_gray+ran_gray_x, (simulator["height"] - (Back_pixels - 3))*debug_scale_gray - 1+ran_gray_y),
                      gray_color["robot_padding"],
                      -1)
        #cv2.rectangle(self.frame_gray,
        #              (simulator["center"]*debug_scale_gray, (simulator["height"] - (Back_pixels + 0))*debug_scale_gray + 1),
        #              ((simulator["center"] + 1)*debug_scale_gray, (simulator["height"] - (Back_pixels - 1))*debug_scale_gray - 1),
        #              gray_color["robot"],
        #              -1)
        cv2.rectangle(self.frame_gray,(simulator["center"]*debug_scale_gray+ran_gray_x,(simulator["height"]-(Back_pixels+0))*debug_scale_gray+1+ran_gray_y),\
                     ((simulator["center"]+1)*debug_scale_gray+ran_gray_x,(simulator["height"]-(Back_pixels-1))*debug_scale_gray-1+ran_gray_y),gray_color["robot_blue"] if self.sorting_plate_state == sorting_plate_state_dic['BLUE'] else gray_color["robot_red"],-1)
        ### gray color of the rollers
        cv2.rectangle(self.frame_gray,
                      ((simulator["center"] - 1)*debug_scale_gray+ran_gray_x, (simulator["height"] - (Back_pixels + 2))*debug_scale_gray+ran_gray_y ),
                      ((simulator["center"] + 2)*debug_scale_gray+ran_gray_x, (simulator["height"] - (Back_pixels + 1))*debug_scale_gray +ran_gray_y),
                      gray_color["robot_roller"],
                      -1)


        scaleup = cv2.resize(self.frame_gray, (simulator["height"]*debug_scale_gray*15,simulator["width"]*debug_scale_gray*15), interpolation=cv2.INTER_NEAREST)

        # Draw the grid
        for i in range(1, simulator["width"]*debug_scale_gray):
            # Vertical grid
            cv2.line(scaleup,
                     (i*debug_scale_gray*15, 0),
                     (i*debug_scale_gray*15, simulator["width"]*debug_scale_gray*15),
                     (128, 128, 128),
                     1)
            # Horizontal grid
            cv2.line(scaleup,
                     (0, i*debug_scale_gray*15),
                     (simulator["width"]*debug_scale_gray*15, i*debug_scale_gray*15),
                     (128, 128, 128),
                     1)

        cv2.imshow("scaleup",scaleup)
        cv2.imwrite( "test.jpg", scaleup )
        cv2.waitKey(5000)

        return self.frame_gray

    def draw_state(self):
        self.frame = np.zeros((simulator["height"],simulator["width"],1), np.uint8)
        for obstacle in self.obstacles:
            cx = simulator["center"] + int(round(1.0*obstacle[0]/trans_scale))
            cy = simulator["height"] - Back_pixels - int(round(1.0*obstacle[1]/trans_scale))
            if self.check_window_state(cx, cy):
                self.frame[cx][cy] = self._params["Map.data.obstacle"]
        for r_ball in self.red_balls:
            if self.state_blink == False or random.random() > (0.1 + 0.2*r_ball[1]/3.0/(map_param["height"]/2)):
                if r_ball[1] >= int(ball_blind_ratio*(abs(1.0*r_ball[0])-ball_blind_bias)) and r_ball[1] >= non_detect:
                    r_ball_x = r_ball[0]
                    r_ball_y = r_ball[1]
                    if self.state_inaccurate:
                        r_ball_x = r_ball_x + 0.5*random.random()*map_param["center"]*(0.1*r_ball_x*r_ball_x/map_param["center"]/map_param["center"] - 0.05)
                        r_ball_y = r_ball_y + 0.5*random.random()*map_param["center"]*(0.1*r_ball_y*r_ball_y/map_param["center"]/map_param["center"] - 0.05)
                    cx = simulator["center"] + int(round(1.0*r_ball_x/trans_scale))
                    cy = simulator["height"] - Back_pixels - int(round(1.0*r_ball_y/trans_scale))
                    if self.check_window_state(cx, cy):
                        self.frame[cx][cy] = self._params["Map.data.red_ball"]

        for b_ball in self.blue_balls:
            if self.state_blink == False or random.random() > (0.1 + 0.2*b_ball[1]/3.0/(map_param["height"]/2)):
                if b_ball[1] >= int(ball_blind_ratio*(abs(1.0*b_ball[0])-ball_blind_bias)) and b_ball[1] >= non_detect:
                    b_ball_x = b_ball[0]
                    b_ball_y = b_ball[1]
                    if self.state_inaccurate:
                        b_ball_x = b_ball_x + 0.5*random.random()*map_param["center"]*(0.1*b_ball_x*b_ball_x/map_param["center"]/map_param["center"] - 0.05)
                        b_ball_y = b_ball_y + 0.5*random.random()*map_param["center"]*(0.1*b_ball_y*b_ball_y/map_param["center"]/map_param["center"] - 0.05)
                    cx = simulator["center"] + int(round(1.0*b_ball_x/trans_scale))
                    cy = simulator["height"] - Back_pixels - int(round(1.0*b_ball_y/trans_scale))
                    if self.check_window_state(cx, cy):
                        self.frame[cx][cy] = self._params["Map.data.blue_ball"]

        self.frame[simulator["center"]][simulator["height"]-Back_pixels] = 255

        self.draw_state_gray()

        return self.frame

    def get_reward(self, action):
        reward = 0
        red_balls_temp = []
        blue_balls_temp = []
        pre_action = 0
        if action in range(self.action_space.size):
            pre_action=action
            if pre_action== 5 or pre_action ==6:
                reward -=4

        # reward for red ball
        for i, r_ball in enumerate(self.red_balls):
            if pre_action == 5 or pre_action == 6:
                cx = round(1.0*r_ball[0]/trans_scale)
                cy = round(1.0*r_ball[1]/trans_scale)
                if cy>=0 and cy < reward_region_y and r_ball[1] >= int(ball_blind_ratio*(abs(1.0*r_ball[0])-ball_blind_bias)-2) and (cx in reward_region_x):
                    if cy >= reward_region_y/3  and cy < 2*reward_region_y/3:
                        if cx == reward_region_x[1]:
                            reward += 20
                        elif cx == reward_region_x[0] or cx == reward_region_x[2]:
                            reward += 10
                        else:
                            reward += -5
                        if len(self.red_balls_prev) > 0 and int(round(1.0*self.red_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                            self.sorting_plate_state != sorting_plate_state_dic['RED']:
                            reward += -10
                    else:
                        if cx == reward_region_x[1]:
                            reward += 10
                        elif cx == reward_region_x[0] or cx == reward_region_x[2]:
                            reward += 5
                        else: 
                            reward += -5
                        if len(self.red_balls_prev) > 0 and int(round(1.0*self.red_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                            self.sorting_plate_state != sorting_plate_state_dic['RED']:
                            reward += -10
                else :
                    red_balls_temp.append(r_ball)
            else:
                cx = round(1.0*r_ball[0]/trans_scale)
                cy = round(1.0*r_ball[1]/trans_scale)
                if cy < reward_region_y and cy >=0 and r_ball[1] >= int(ball_blind_ratio*(abs(1.0*r_ball[0])-ball_blind_bias)-2) and (cx in reward_region_x):
                    reward += -5
                    if len(self.red_balls_prev) > 0 and int(round(1.0*self.red_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                        self.sorting_plate_state != sorting_plate_state_dic['RED']:
                        reward += -15
                else:
                    red_balls_temp.append(r_ball)
        
        # reward for blue ball
        for i, b_ball in enumerate(self.blue_balls):
            if pre_action == 5 or pre_action == 6:
                cx = round(1.0*b_ball[0]/trans_scale)
                cy = round(1.0*b_ball[1]/trans_scale)
                if cy>=0 and cy < reward_region_y and b_ball[1] >= int(ball_blind_ratio*(abs(1.0*b_ball[0])-ball_blind_bias)-2) and (cx in reward_region_x):
                    if cy >=reward_region_y/3  and cy < 2*reward_region_y/3:
                        if cx == reward_region_x[1]:
                            reward += 20
                        elif cx == reward_region_x[0] or cx == reward_region_x[2]:
                            reward += 10
                        else:
                            reward += -5
                        if len(self.blue_balls_prev) > 0 and int(round(1.0*self.blue_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                            self.sorting_plate_state != sorting_plate_state_dic['BLUE']:
                            reward += -10
                    else: 
                        if cx == reward_region_x[1]:
                            reward += 10
                        elif cx == reward_region_x[0] or cx == reward_region_x[2]:
                            reward += 5
                        else: 
                            reward += -5
                        if len(self.blue_balls_prev) > 0 and int(round(1.0*self.blue_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                            self.sorting_plate_state != sorting_plate_state_dic['BLUE']:
                            reward += -10
                else:
                    blue_balls_temp.append(b_ball)
            else:
                cx = round(1.0*b_ball[0]/trans_scale)
                cy = round(1.0*b_ball[1]/trans_scale)
                if cy < reward_region_y and cy >=0 and b_ball[1] >= int(ball_blind_ratio*(abs(1.0*b_ball[0])-ball_blind_bias)-2) and (cx in reward_region_x):
                    reward += -5
                    if len(self.blue_balls_prev) > 0 and int(round(1.0*self.blue_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                        self.sorting_plate_state != sorting_plate_state_dic['BLUE']:
                        reward += -15
                else:
                    blue_balls_temp.append(b_ball)

        self.red_balls = red_balls_temp
        self.blue_balls = blue_balls_temp

        red_balls_inscreen = []
        blue_balls_inscreen = []
        robots_dis_cor=[]
        robots_dis_cen=[]
        for r_ball in red_balls_temp:
            if r_ball[1] >= ball_blind_ratio * (abs(1.0*r_ball[0]) - ball_blind_bias)\
                and abs(1.0*r_ball[0]) <= map_param["center"] and abs(1.0*r_ball[1]) < map_param["height"]:
                red_balls_inscreen.append(r_ball)
        for b_ball in blue_balls_temp:
            if b_ball[1] >= ball_blind_ratio * (abs(1.0*b_ball[0]) - ball_blind_bias)\
                and abs(1.0*b_ball[0]) <= map_param["center"] and abs(1.0*b_ball[1]) < map_param["height"]:
                blue_balls_inscreen.append(b_ball)

        # What is this for?
        if action in range(self.action_space.size):
            if len(red_balls_inscreen) == 0 and len(blue_balls_inscreen) == 0:
                self.ball_inscreen_flag = self.ball_inscreen_flag + 1
                if action == 3:
                    reward += 0.7
            else:
                self.ball_inscreen_flag = 0
        #distance robots from right above corner and distance robots from center of map /
        # if 2 variable changes smaller than "10" in 3-steps reward-=0.3
        if action in range(self.action_space.size):
            robots_dis_cen=np.sqrt((self.robots_cen[0][0])*(self.robots_cen[0][0])+(self.robots_cen[0][1])*(self.robots_cen[0][1]))
            if len(self.state_dis_cen) < 4 :
                self.state_dis_cen.append(robots_dis_cen)
            else :
                self.state_dis_cen[0]=self.state_dis_cen[1]
                self.state_dis_cen[1]=self.state_dis_cen[2]
                self.state_dis_cen[2]=self.state_dis_cen[3]
                self.state_dis_cen[3]=robots_dis_cen
            robots_dis_cor=np.sqrt((self.robots_cor[0][0])*(self.robots_cor[0][0])+(self.robots_cor[0][1])*(self.robots_cor[0][1]))
            if len(self.state_dis_cor) < 4 :
                self.state_dis_cor.append(robots_dis_cor)
            else :
                self.state_dis_cor[0]=self.state_dis_cor[1]
                self.state_dis_cor[1]=self.state_dis_cor[2]
                self.state_dis_cor[2]=self.state_dis_cor[3]
                self.state_dis_cor[3]=robots_dis_cor
            if len(self.state_dis_cen) == 4 and len(self.state_dis_cor) == 4 :
                dcen=self.state_dis_cen[3]-self.state_dis_cen[0]
                dcor=self.state_dis_cor[3]-self.state_dis_cor[0]
                if dcen*dcen < 10 and dcor*dcor < 10 :
                    reward -= 0.5

        if self.iter > max_iter or (len(red_balls_temp) == 0 and len(blue_balls_temp) == 0) : #or self.ball_inscreen_flag >= 10
            self.done = True
            if len(red_balls_temp) == 0 and len(blue_balls_temp) == 0:
                #reward+=(max_iter-step_reward)/2
                #print(reward)
                print('len(red_balls_temp) == 0 and len(blue_balls_temp) == 0')
            if self.iter > max_iter:
                print('self.iter > max_iter')
            #if self.ball_inscreen_flag >= 10:
               # print('self.ball_inscreen_flag >= 10')

        if self.done:
            self.episode_rewards.append(self.score)
            if self.write_flag:
                self.video.release()
                print ("video saved")

        if action == -1:
            return -1  # Penalize if not able to move (Try -2 ?)
        else:
            return reward

    def step(self, action):
        if action in range(self.action_space.size):
            self.iter = self.iter + 1

        del_x, del_y, rot = 0, 0, 0

        if action == 0: # forward
            del_x, del_y = 0, -1
        #elif action == 1: # forward right
        #    del_x, del_y = -1, -1
        elif action == 1: # right
            del_x, del_y = -1, 0
        #elif action == 3: # backward right
        #    del_x, del_y = -1, 1
        #elif action == 7: # backward
            #del_x, del_y = 0, 1
        #elif action == 5: # bacward left
        #    del_x, del_y = 1, 1
        elif action == 2: # left
            del_x, del_y = 1, 0
        #elif action == 7: # forward left
            #del_x, del_y = 1, -1
        elif action == 4: # turn left
            rot = -1
        elif action == 3: # turn right
            rot = 1
        elif action == 5:
            del_x, del_y, self.sorting_plate_state = 0, -3, sorting_plate_state_dic['RED']
        elif action == 6:
            del_x, del_y, self.sorting_plate_state = 0, -3, sorting_plate_state_dic['BLUE']
        else:
            del_x, del_y, rot = 0, 0, 0

        red_balls_temp = []
        blue_balls_temp = []
        obstacles_temp = []
        robots_cen_temp=[]
        robots_cor_temp=[]

        del_x = del_x * trans_scale
        del_y = del_y * trans_scale

        if len(self.red_balls) > 0:
            red_balls_temp = np.add(self.red_balls, [del_x,del_y])

        if len(self.blue_balls) > 0:
            blue_balls_temp = np.add(self.blue_balls, [del_x,del_y])

        if len(self.obstacles) > 0:
            obstacles_temp = np.add(self.obstacles, [del_x,del_y])

        if len(self.robots_cen) > 0:
            robots_cen_temp = np.add(self.robots_cen, [del_x,del_y])      

        if len(self.robots_cor) > 0:
            robots_cor_temp = np.add(self.robots_cor, [del_x,del_y])        

        if action == 3 or action == 4:
            points = np.concatenate([x for x in [red_balls_temp, blue_balls_temp, obstacles_temp, robots_cen_temp,robots_cor_temp] if len(x) > 0])
            if points.size > 0:
                points = points.reshape(-1,2)
                theta = rot_scale*rot*np.pi/180
                theta_0 = np.arctan2(points.T[1],points.T[0])

                ball_dist = np.linalg.norm(points, axis=1)
                rot_delta_unit_x = np.subtract(np.cos(theta_0), np.cos(np.add(theta_0,theta)))
                rot_delta_unit_y = np.subtract(np.sin(theta_0), np.sin(np.add(theta_0,theta)))
                rot_delta_unit = np.concatenate((rot_delta_unit_x.reshape(-1,1),rot_delta_unit_y.reshape(-1,1)),axis=1)
                ball_dist = np.concatenate((ball_dist.reshape(-1,1),ball_dist.reshape(-1,1)),axis=1)
                rot_delta = np.multiply(ball_dist, rot_delta_unit)
                points = np.subtract(points, rot_delta)
                red_balls_temp = points[0:len(self.red_balls)]
                blue_balls_temp = points[len(self.red_balls):len(self.red_balls)+len(self.blue_balls)]
                obstacles_temp = points[len(self.red_balls)+len(self.blue_balls):len(self.red_balls)+len(self.blue_balls)+len(self.obstacles)]
                robots_cen_temp= points[len(self.red_balls)+len(self.blue_balls)+len(self.obstacles):len(self.red_balls)+len(self.blue_balls)+len(self.obstacles)+len(self.robots_cen)]
                robots_cor_temp=points[len(self.red_balls)+len(self.blue_balls)+len(self.obstacles)+len(self.robots_cen):]
        enable_move = True
        for obstacle in obstacles_temp:
            if abs(1.0*obstacle[0]) < 8.0*trans_scale/3 and abs(1.0*obstacle[1]) < 8.0*trans_scale/3:
                enable_move = False

        reward = 0
        if enable_move:
            self.red_balls = red_balls_temp
            self.blue_balls = blue_balls_temp
            self.robots_cen = robots_cen_temp
            self.robots_cor = robots_cor_temp
            reward = self.get_reward(action)
            self.obstacles = obstacles_temp
            self.draw_state()
            self.red_balls_prev = self.red_balls
            self.blue_balls_prev = self.blue_balls
        else:
            reward = self.get_reward(-1)

        self.score = self.score + reward
        self.current_reward = reward

        if self.write_flag:
            frame_debug = self.draw_debug_frame(self.frame)
            self.video.write(frame_debug)

        if self.debug_flag:
            frame_debug = self.draw_debug_frame(self.frame)
            cv2.imshow("frame_debug", frame_debug)
            cv2.imshow("frame_debug_gray", self.frame_gray)
            cv2.waitKey(100)

        return self.frame_gray, reward, self.done

    def get_total_steps(self):
        return self.iter

    def get_episode_rewards(self):
        return self.episode_rewards

    def action_space_sample(self):
        index = int(1.0*random.random()*self.action_space.size)
        return self.action_space[index]


if __name__ == '__main__':
    tk = Task(debug_flag=True, test_flag=False, state_blink=True, state_inaccurate=True)
    tk.reset()

    action = -1
    while(1):
        tk.step(action)
        key = cv2.waitKey(300)&0xFF
        action = -1
        if key == ord('q') or tk.done == True:
            break
        elif key == ord('w'):
            action = 0
        elif key == ord('d'):
            action = 1
        #elif key == ord('s'):
            #action = 7
        elif key == ord('a'):
            action = 2
        elif key == ord('z'):
            action = 4
        elif key == ord('c'):
            action = 3
        elif key == ord('1'):
            action = 5
        elif key == ord('2'):
            action = 6

    print("shutdown")
    # cv2.destroyAllWindows()
