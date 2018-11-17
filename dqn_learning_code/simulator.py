#!/usr/bin/env python3
import cv2
import numpy as np
import yaml, sys, time, random
import math
# import rospy, roslib, rospkg

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Vector3
# from tt_core_msgs.msg import Vector3DArray, ImagePoint
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import CompressedImage

# Simulator -----------------------------------------------------------------------------------
simulator = {"width":31, "height":31, "center":15, "resol":2}  # Will be used as input to the policy network
# resol = # Zoom out the view

map_param = {"width":30, "height":50, "center":25, "resol":1, "scale":5}  # Size of the map (Should be same with real one?)
# resol = 
# scale = ???

trans_scale = int(simulator["resol"]/map_param["resol"])  # Zoom out the view

# Settings for simulator ----------------------------------------------------------------------
back_pixels = 4  # number of pixels behind (In simulator scale)
back_pixels_map = back_pixels*trans_scale  # (In map scale)

debug_scale = 10  # Scale up debug window for visualization
DEBUG_SCALE_WIT = 10  # Scale up my debug window
DEBUG_SCALE_ENV = 10
debug_scale_gray = 10

# Robot parameters ----------------------------------------------------------------------------
# In map scale
#
#              COLLECT_X
#              |       |
#       _______________       ___
#      |    collect    |
#      |    region     |   COLLECT_Y        Not finished yet
#      |_______________|      ___
#      +---------------+
#      |               |
#      |               |
#      |               |
#      |       +       |      ---
#      |               |
#      |               |      R_Y
#      |               |
#      +---------------+      ---
#              |  R_X  |

rot_scale = 5  # Rotation step (in degree)
camera_fov = 78  # In degree
ball_blind_slope = 1/np.tan(camera_fov/2*np.pi/180)
ball_blind_bias = 1  # In simulator scale
ball_blind_bias_map = 1.5  # in map scale
x_disable_move = 4  # In map scale (Shown using green line)
y_disable_move = 5  # in map scale
R_BALL = 0.75
R_X = 1.5
R_Y = 2.5
COLLECT_X = 1.5
COLLECT_Y = 1

## 5cm per 1 map pixel  ## when simulator resol is 3, 15cm per 1 simulator pixel 
margin = 4 ## MARGIN 
ball_margin = 10
Ran_wall = 20 # random lengthen wall size (늘어난 벽의 길이)
obstacle_base = 2/5*map_param["height"] ## 
obstacle_length = int(2/5*map_param["width"]) ##

# DQN -----------------------------------------------------------------------------------------
max_iter = 1000  # Why only 99?
reward_region_x = [-1,0,1]  # In simulator scale
reward_region_y = 3  # In simulator scale

# We also want to train the sorting plate so that it can move properly when it detect the color of the ball
sorting_plate_state_dic = {'NULL': 0, 'RED': 1, 'BLUE': 2}

class Task:
    def __init__(self, debug_flag=False, test_flag=False, state_blink=True, state_inaccurate=True):
        # This is in simulator scale
        self.frame = np.zeros((simulator["height"], simulator["width"], 1), np.uint8)
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray, simulator["width"]*debug_scale_gray, 1), np.uint8)
        # This is in map scale
        self.balls = []
        self.index_prev = -1  # Add by myself
        self.red_balls = []
        self.blue_balls = []
        self.red_balls_prev = []
        self.blue_balls_prev = []
        self.obstacles = []
        # For ...
        self.episode_rewards = []
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
        self.action_space = np.array(range(12))
        # ROS
        # rospack = rospkg.RosPack()
        # root = rospack.get_path('tt_rl_motion_planner')
        # path = root+"/config/map_gen.yaml"
        path = "./map_gen.yaml"
        stream = open(path, 'r')
        self._params = yaml.load(stream)
        # Add by myself
        self.current_reward = 0
        return

    def reset(self, max_balls=10, max_walls=2):
        self.frame = np.zeros((simulator["height"], simulator["width"], 1), np.uint8)
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray, simulator["width"]*debug_scale_gray, 1), np.uint8)
        self.balls = []
        self.index_prev = -1  # Add by myself
        self.red_balls = []
        self.blue_balls = []
        self.red_balls_prev = []
        self.blue_balls_prev = []
        self.obstacles = []
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
            out_directory = "data/video/tt.video."+format(len(self.episode_rewards)/200,"08")+".mp4"

        if self.test_flag:
            self.write_flag = True
            out_directory = "data/video_test/tt.video."+format(len(self.episode_rewards),"08")+".mp4"

        if self.write_flag:
            codec = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 10
            self.video = cv2.VideoWriter(out_directory, codec, fps, (simulator["width"]*debug_scale,simulator["height"]*debug_scale))

        #       map_param                               simulator
        #           ^
        #           | y
        # +-------------------+
        # |    \    |         |
        # |     \i_t|         |                               +----------------+
        # |      \  |         | w_h                           |                |
        # |       \ |         |                               |                |
        # |        \|         |                               |                |
        # |         +---------|-----> x     +-----------+     |                |
        # |         |         |             |           |     |   frame_gray   |
        # |         |         |             |           |     |                |
        # |         |         |             |   frame   |     |                |
        # |         |         |             |           |     |                |
        # |         |         |             |           |     |                |
        # +-------------------+             +-----------+     +----------------+
        #          w_w                                     = frame X debug_scale_gray

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

        # Force debug ------------------------------------------------------------------------
        i_t = 0

        # if rand_map <= 0.333:
        #     # map without walls nor obstacles
        #     w_w = map_param["width"]
        #     w_h = map_param["height"]
        #     r_x = (random.random()-0.5)*(w_w - 2*margin) # initial robot rx map base
        #     r_y = -(w_h-2*margin)/2 + random.random()*(obstacle_base-2*margin) # initial robot ry map base
        #     walls_initial = []
        # else:
        # if rand_map >= 0.666:
        #     # map with walls only
        #     w_w = map_param["width"] + round(random.random()*Ran_wall) # random wall width
        #     w_h = map_param["height"] + round(random.random()*Ran_wall) # random wall height
        #     r_x = (random.random()-0.5)*(w_w - 2*margin) # initial robot rx map base
        #     r_y = -(w_h-2*margin)/2 + random.random()*(obstacle_base-2*margin) # initial robot ry map base
        # else:
        # map with walls and obstacles
        w_w = map_param["width"] + round(random.random()*Ran_wall)
        w_h = map_param["height"] + round(random.random()*Ran_wall) + round(obstacle_base)
        r_x = (random.random()-0.5)*(w_w - 2*margin) # initial robot rx map base
        r_y = -(w_h-2*margin)/2 + random.random()*(obstacle_base-2*margin) # initial robot ry map base
        
        # force debug
        r_x = 0
        r_y = 0
        w_w = map_param["width"]
        w_h = map_param["height"]

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
        frame_debug = np.zeros((simulator["height"]*debug_scale, simulator["width"]*debug_scale, 3), np.uint8)
        # For reference

        # cv.rectangle
        # img       Image.
        # pt1       Vertex of the rectangle.
        # pt2       Vertex of the rectangle opposite to pt1 .
        # color     Rectangle color or brightness (grayscale image).
        # thickness Thickness of lines that make up the rectangle.
        #           Negative values, like FILLED, mean that the function has to draw a filled rectangle.

        # cv.line
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
                    # cv2.line(frame_debug,
                    #          ((simulator["center"])*debug_scale - 1, (simulator["height"] - back_pixels)*debug_scale),
                    #          (i*debug_scale, j*debug_scale),
                    #          (0, 0, 255),
                    #          1)
                if frame[i][j] == self._params["Map.data.blue_ball"]:
                    cv2.rectangle(frame_debug,
                                  (i*debug_scale, j*debug_scale),
                                  ((i + 1)*debug_scale - 1, (j + 1)*debug_scale - 1),
                                  (255, 0, 0),
                                  -1)
                    # cv2.line(frame_debug,
                    #          ((simulator["center"])*debug_scale - 1, (simulator["height"] - back_pixels)*debug_scale),
                    #          (i*debug_scale, j*debug_scale),
                    #          (255, 0, 0),
                    #          1)
        
        # Center of the robot
        cv2.rectangle(frame_debug,
                      (simulator["center"]*debug_scale - 1, (simulator["height"] - back_pixels)*debug_scale + 1),
                      ((simulator["center"] + 1)*debug_scale, (simulator["height"] - (back_pixels - 1))*debug_scale - 1),
                      (255, 0, 0) if self.sorting_plate_state == sorting_plate_state_dic['BLUE'] else (0, 0, 255),
                      -1)
        # Boundary of the robot
        cv2.rectangle(frame_debug,
                      ((simulator["center"] - 1)*debug_scale - 1, (simulator["height"] - (back_pixels + 2))*debug_scale + 1),
                      ((simulator["center"] + 2)*debug_scale, (simulator["height"] - (back_pixels - 3))*debug_scale - 1),
                      (0, 0, 255),
                      2)
        # Right line view angle
        cv2.line(frame_debug,
                 ((simulator["center"] + ball_blind_bias)*debug_scale, (simulator["height"] - back_pixels)*debug_scale - 1),
                 (simulator["width"]*debug_scale - 1, ((simulator["height"] - back_pixels) - int(ball_blind_slope*(simulator["center"] - 1 - ball_blind_bias)))*debug_scale),
                 (128, 128, 128),
                 1)
        # Left line view angle
        cv2.line(frame_debug,
                 ((simulator["center"] - ball_blind_bias + 1)*debug_scale, (simulator["height"] - back_pixels)*debug_scale - 1),
                 (0, (simulator["height"] - back_pixels - int(ball_blind_slope*(simulator["center"] - 1 - ball_blind_bias)))*debug_scale),
                 (128, 128, 128),
                 1)
        # Text
        cv2.putText(frame_debug,
                    "Step " + str(self.iter),
                    (int(simulator["width"]*debug_scale*0.05),  int(simulator["width"]*debug_scale*0.05)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*debug_scale,
                    (255, 255, 255))
        cv2.putText(frame_debug,
                    "Score " + str(self.score),
                    (int(simulator["width"]*debug_scale*0.05), int(simulator["width"]*debug_scale*0.10)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*debug_scale,
                    (255, 255, 255))
        cv2.putText(frame_debug,
                    "Reward " + str(self.current_reward),
                    (int(simulator["width"]*debug_scale*0.05),  int(simulator["width"]*debug_scale*0.15)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*debug_scale,
                    (255, 255, 255))

        return frame_debug
    
    def draw_environment(self):
        """
        Draw:
        - Every thing stored in map_param, scaled up with DEBUG_SCALE_WIT
        - Path to the nearest seeable ball
        """
        spare = 2
        frame_env = np.zeros((map_param["height"]*DEBUG_SCALE_ENV*spare, map_param["width"]*DEBUG_SCALE_ENV*spare, 3), np.uint8)
        f_cx = frame_env.shape[1]/2
        f_cy = frame_env.shape[0]/2
        # cv.drawMarker
        # img           Image.
        # position	    The point where the crosshair is positioned.
        # color         Line color.
        # markerType    The specific type of marker you want to use, see MarkerTypes
        # markerSize    The length of the marker axis [default = 20 pixels]
        # thickness     Line thickness.
        # line_type     Type of the line, See LineTypes

        # cv.circle
        # img           Image where the circle is drawn.
        # center	    Center of the circle.
        # radius	    Radius of the circle.
        # color         Circle color.
        # thickness	    Thickness of the circle outline, if positive. Negative values, like FILLED, mean that a filled circle is to be drawn.
        # lineType	    Type of the circle boundary. See LineTypes
        # shift	        Number of fractional bits in the coordinates of the center and in the radius value.
        
        # Vertical grid
        for i in range(1, spare*map_param["width"]):
            cv2.line(frame_env,
                     (i*DEBUG_SCALE_ENV, 0),
                     (i*DEBUG_SCALE_ENV, map_param["height"]*DEBUG_SCALE_ENV*spare - 1),
                     (30, 30, 30),
                     1)
        # Horizontal grid
        for i in range(1, spare*map_param["height"]):
            cv2.line(frame_env,
                     (0, i*DEBUG_SCALE_ENV),
                     (map_param["width"]*DEBUG_SCALE_ENV*spare - 1, i*DEBUG_SCALE_ENV),
                     (30, 30, 30),
                     1)
        # Center of the robot
        cv2.drawMarker(frame_env,
                       (int(round(f_cx)), int(round(f_cy))),
                       (0, 255, 0),
                       cv2.MARKER_CROSS,
                       10)
        # Movable region
        cv2.rectangle(frame_env,
                      (int(round(f_cx + R_X*DEBUG_SCALE_ENV)), int(round(f_cy - R_Y*DEBUG_SCALE_ENV))),
                      (int(round(f_cx - R_X*DEBUG_SCALE_ENV)), int(round(f_cy + R_Y*DEBUG_SCALE_ENV))),
                      (0, 255, 0),
                      1,
                      1)
        # Right line view angle
        cv2.line(frame_env,
                 (int(round(f_cx + (ball_blind_bias_map)*DEBUG_SCALE_ENV)), int(round(f_cy))),
                 (frame_env.shape[1], int(round(f_cy - ball_blind_slope*(f_cx - ball_blind_bias_map*DEBUG_SCALE_ENV)))),
                 (128, 128, 128),
                 1)
        # Left line view angle
        cv2.line(frame_env,
                 (int(round(f_cx - (ball_blind_bias_map)*DEBUG_SCALE_ENV)), int(round(f_cy))),
                 (0, int(round(f_cy - ball_blind_slope*(f_cx - ball_blind_bias_map*DEBUG_SCALE_ENV)))),
                 (128, 128, 128),
                 1)
        # Wall
        for obstacle in self.obstacles:
            cv2.drawMarker(frame_env,
                           (int(round(f_cx + (obstacle[0])*DEBUG_SCALE_ENV)), int(round(f_cy - (obstacle[1])*DEBUG_SCALE_ENV))),
                           (255, 255, 0),
                           cv2.MARKER_DIAMOND,
                           5,
                           1)
        # Red balls
        for ball in self.red_balls:
            cv2.circle(frame_env,
                       (int(round(f_cx + ball[0]*DEBUG_SCALE_ENV)), int(round(f_cy - (ball[1])*DEBUG_SCALE_ENV))),
                       int(round(R_BALL*DEBUG_SCALE_ENV)),
                       (0, 0, 255),
                       -1)
        # Blue balls
        for ball in self.blue_balls:
            cv2.circle(frame_env,
                       (int(round(f_cx + ball[0]*DEBUG_SCALE_ENV)), int(round(f_cy - (ball[1])*DEBUG_SCALE_ENV))),
                       int(round(R_BALL*DEBUG_SCALE_ENV)),
                       (255, 0, 0),
                       -1)
        
        ball_list = self.red_balls + self.blue_balls
        if ball_list:
            ball_distances = np.sum(np.asarray(ball_list)**2, axis=1)
            sorted_indices = np.argsort(ball_distances)
            for index in sorted_indices:
                if in_camera_range(ball_list[index]) and no_wall_blocking([0, 0], ball_list[index], self.obstacles):
                    if index < len(self.red_balls):
                        cv2.line(frame_env,
                                 (int(round(f_cx)), int(round(f_cy))),
                                 (int(round(f_cx + self.red_balls[index][0]*DEBUG_SCALE_ENV)), int(round(f_cy - self.red_balls[index][1]*DEBUG_SCALE_ENV))),
                                 (0, 0, 255),
                                 1)
                        break
                    else:
                        index = index - len(self.red_balls)
                        cv2.line(frame_env,
                                 (int(round(f_cx)), int(round(f_cy))),
                                 (int(round(f_cx + self.blue_balls[index][0]*DEBUG_SCALE_ENV)), int(round(f_cy - self.blue_balls[index][1]*DEBUG_SCALE_ENV))),
                                 (255, 0, 0),
                                 1)
                        break
        
        # Text --------------------------------------------------------------------------------
        cv2.putText(frame_env,
                    "Step " + str(self.iter),
                    (int(map_param["width"]*DEBUG_SCALE_ENV*0.05),  int(map_param["width"]*DEBUG_SCALE_ENV*0.05)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*DEBUG_SCALE_ENV,
                    (255, 255, 255))
        cv2.putText(frame_env,
                    "Score " + str(self.score),
                    (int(map_param["width"]*DEBUG_SCALE_ENV*0.05), int(map_param["width"]*DEBUG_SCALE_ENV*0.10)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*DEBUG_SCALE_ENV,
                    (255, 255, 255))
        cv2.putText(frame_env,
                    "Reward " + str(self.current_reward),
                    (int(map_param["width"]*DEBUG_SCALE_ENV*0.05),  int(map_param["width"]*DEBUG_SCALE_ENV*0.15)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.05*DEBUG_SCALE_ENV,
                    (255, 255, 255))
        
        return frame_env
    
    def draw_debug_wit(self):
        """
        Draw:
        - Every thing stored in map_param, scaled up with DEBUG_SCALE_WIT
        - Path to the nearest seeable ball
        """
        frame_env = np.zeros((map_param["height"]*DEBUG_SCALE_WIT, map_param["width"]*DEBUG_SCALE_WIT, 3), np.uint8)
        # cv.drawMarker
        # img           Image.
        # position	    The point where the crosshair is positioned.
        # color         Line color.
        # markerType    The specific type of marker you want to use, see MarkerTypes
        # markerSize    The length of the marker axis [default = 20 pixels]
        # thickness     Line thickness.
        # line_type     Type of the line, See LineTypes

        # cv.circle
        # img           Image where the circle is drawn.
        # center	    Center of the circle.
        # radius	    Radius of the circle.
        # color         Circle color.
        # thickness	    Thickness of the circle outline, if positive. Negative values, like FILLED, mean that a filled circle is to be drawn.
        # lineType	    Type of the circle boundary. See LineTypes
        # shift	        Number of fractional bits in the coordinates of the center and in the radius value.
        
        # Draw the grid
        for i in range(1, map_param["width"]):
            # Vertical grid
            cv2.line(frame_env,
                     (i*DEBUG_SCALE_WIT, 0),
                     (i*DEBUG_SCALE_WIT, map_param["height"]*DEBUG_SCALE_WIT - 1),
                     (30, 30, 30),
                     1)
            # Horizontal grid
            cv2.line(frame_env,
                     (0, i*DEBUG_SCALE_WIT),
                     (map_param["width"]*DEBUG_SCALE_WIT - 1, i*DEBUG_SCALE_WIT),
                     (30, 30, 30),
                     1)
        # Center of the robot
        cv2.drawMarker(frame_env,
                       ((map_param["center"])*DEBUG_SCALE_WIT, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
                       (255, 255, 0),
                       cv2.MARKER_CROSS,
                       10)
        # Movable region
        cv2.rectangle(frame_env,
                      (int(round((map_param["center"] - (R_X))*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (back_pixels_map + R_Y))*DEBUG_SCALE_WIT))),
                      (int(round((map_param["center"] + (R_X))*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (back_pixels_map - R_Y))*DEBUG_SCALE_WIT))),
                      (0, 255, 0),
                      1,
                      1)
        # Right line view angle
        cv2.line(frame_env,
                 ((map_param["center"] + ball_blind_bias_map)*DEBUG_SCALE_WIT, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
                 (map_param["width"]*DEBUG_SCALE_WIT, int(round((map_param["height"] - (back_pixels_map + ball_blind_slope*(map_param["center"] - ball_blind_bias_map)))*DEBUG_SCALE_WIT))),
                 (128, 128, 128),
                 1)
        # Left line view angle
        cv2.line(frame_env,
                 ((map_param["center"] - ball_blind_bias_map)*DEBUG_SCALE_WIT, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
                 (0, int(round((map_param["height"] - (back_pixels_map + ball_blind_slope*(map_param["center"] - ball_blind_bias_map)))*DEBUG_SCALE_WIT))),
                 (128, 128, 128),
                 1)
        # Wall
        for obstacle in self.obstacles:
            cv2.drawMarker(frame_env,
                           (int(round((map_param["center"] + (obstacle[0]))*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (obstacle[1] + back_pixels_map))*DEBUG_SCALE_WIT))),
                           (255, 255, 0),
                           cv2.MARKER_DIAMOND,
                           5,
                           1)
        # Red balls
        for ball in self.red_balls:
            cv2.circle(frame_env,
                       (int(round((map_param["center"] + (ball[0]))*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (ball[1] + back_pixels_map))*DEBUG_SCALE_WIT))),
                       1*DEBUG_SCALE_WIT,
                       (0, 0, 255),
                       -1)
            # cv2.line(frame_env,
            #          ((map_param["center"])*debug_scale, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
            #          (int(round((map_param["center"] + ball[0])*debug_scale)), int(round((map_param["height"] - (back_pixels_map + ball[1]))*debug_scale))),
            #          (0, 0, 255),
            #          1)
        # Blue balls
        for ball in self.blue_balls:
            cv2.circle(frame_env,
                       (int(round((map_param["center"] + (ball[0]))*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (ball[1] + back_pixels_map))*DEBUG_SCALE_WIT))),
                       1*DEBUG_SCALE_WIT,
                       (255, 0, 0),
                       -1)
            # cv2.line(frame_env,
            #          ((map_param["center"])*debug_scale, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
            #          (int(round((map_param["center"] + ball[0])*debug_scale)), int(round((map_param["height"] - (back_pixels_map + ball[1]))*debug_scale))),
            #          (255, 0, 0),
            #          1)
        
        ball_list = self.red_balls + self.blue_balls
        if ball_list:
            ball_distances = np.sum(np.asarray(ball_list)**2, axis=1)
            sorted_indices = np.argsort(ball_distances)
            for index in sorted_indices:
                if in_camera_range(ball_list[index]) and no_wall_blocking([0, 0], ball_list[index], self.obstacles):
                    if index < len(self.red_balls):
                        cv2.line(frame_env,
                                 ((map_param["center"])*DEBUG_SCALE_WIT, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
                                 (int(round((map_param["center"] + self.red_balls[index][0])*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (back_pixels_map + self.red_balls[index][1]))*DEBUG_SCALE_WIT))),
                                 (0, 0, 255),
                                 1)
                        break
                    else:
                        index = index - len(self.red_balls)
                        cv2.line(frame_env,
                                 ((map_param["center"])*DEBUG_SCALE_WIT, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
                                 (int(round((map_param["center"] + self.blue_balls[index][0])*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (back_pixels_map + self.blue_balls[index][1]))*DEBUG_SCALE_WIT))),
                                 (255, 0, 0),
                                 1)
                        break

        # Can use funciton, but it's too slow
        # index = get_nearest_seeable_ball_index([0, 0], self.red_balls + self.blue_balls, self.obstacles)
        # if index != -1:
        #     if index < len(self.red_balls):
        #         print('Draw red')
        #         cv2.line(frame_env,
        #                 ((map_param["center"])*DEBUG_SCALE_WIT, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
        #                 (int(round((map_param["center"] + self.red_balls[index][0])*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (back_pixels_map + self.red_balls[index][1]))*DEBUG_SCALE_WIT))),
        #                 (0, 0, 255),
        #                 1)
        #     else:
        #         index = index - len(self.red_balls)
        #         print('Draw blue')
        #         cv2.line(frame_env,
        #                 ((map_param["center"])*DEBUG_SCALE_WIT, (map_param["height"] - back_pixels_map)*DEBUG_SCALE_WIT),
        #                 (int(round((map_param["center"] + self.blue_balls[index][0])*DEBUG_SCALE_WIT)), int(round((map_param["height"] - (back_pixels_map + self.blue_balls[index][1]))*DEBUG_SCALE_WIT))),
        #                 (255, 0, 0),
        #                 1)
        
        # Text --------------------------------------------------------------------------------
        cv2.putText(frame_env,
                    "Step " + str(self.iter),
                    (int(map_param["width"]*DEBUG_SCALE_WIT*0.05),  int(map_param["width"]*DEBUG_SCALE_WIT*0.05)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.1*DEBUG_SCALE_WIT,
                    (255, 255, 255))
        cv2.putText(frame_env,
                    "Score " + str(self.score),
                    (int(map_param["width"]*DEBUG_SCALE_WIT*0.05), int(map_param["width"]*DEBUG_SCALE_WIT*0.10)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.1*DEBUG_SCALE_WIT,
                    (255, 255, 255))
        cv2.putText(frame_env,
                    "Reward " + str(self.current_reward),
                    (int(map_param["width"]*DEBUG_SCALE_WIT*0.05),  int(map_param["width"]*DEBUG_SCALE_WIT*0.15)),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    0.1*DEBUG_SCALE_WIT,
                    (255, 255, 255))
        
        return frame_env

    def draw_state_gray(self):
        gray_color = {"red_ball":255, "blue_ball":220, "wall":100, "robot":200, "robot_padding":150}
        # 다음 교육에는 sorting plate 상태에 따라 gray에도 표시해줌
        # gray_color = {"red_ball":255, "blue_ball":220, "wall":100, "robot_red":200, "robot_blue":180, "robot_padding":150}
        self.frame_gray = np.zeros((simulator["height"]*debug_scale_gray, simulator["width"]*debug_scale_gray, 1), np.uint8)

        for i in range(simulator["width"]):
            for j in range(simulator["height"]):
                if self.frame[i][j] == self._params["Map.data.obstacle"]:
                    cv2.rectangle(self.frame_gray,
                                  (i*debug_scale_gray, j*debug_scale_gray),
                                  ((i + 1)*debug_scale_gray - 1, (j + 1)*debug_scale_gray - 1),
                                  gray_color["wall"],
                                  -1)
                if self.frame[i][j] == self._params["Map.data.red_ball"]:
                    cv2.rectangle(self.frame_gray,
                                  (i*debug_scale_gray, j*debug_scale_gray),
                                  ((i + 1)*debug_scale_gray - 1, (j + 1)*debug_scale_gray - 1),
                                  gray_color["red_ball"],
                                  -1)
                if self.frame[i][j] == self._params["Map.data.blue_ball"]:
                    cv2.rectangle(self.frame_gray,
                                  (i*debug_scale_gray, j*debug_scale_gray),
                                  ((i + 1)*debug_scale_gray - 1, (j + 1)*debug_scale_gray - 1),
                                  gray_color["blue_ball"],
                                  -1)
        # Boundary of the robot (?)
        cv2.rectangle(self.frame_gray,
                      ((simulator["center"] - 1)*debug_scale_gray, (simulator["height"] - (back_pixels + 2))*debug_scale_gray + 1),
                      ((simulator["center"] + 2)*debug_scale_gray, (simulator["height"] - (back_pixels - 3))*debug_scale_gray - 1),
                      gray_color["robot_padding"],
                      -1)
        # Center of the robot
        cv2.rectangle(self.frame_gray,
                      (simulator["center"]*debug_scale_gray, (simulator["height"] - (back_pixels + 0))*debug_scale_gray + 1),
                      ((simulator["center"] + 1)*debug_scale_gray, (simulator["height"] - (back_pixels - 1))*debug_scale_gray - 1),
                      gray_color["robot"],
                      -1)
        
        # cv2.rectangle(self.frame_gray,
        #               (simulator["center"]*debug_scale_gray, (simulator["height"] - (back_pixels + 0))*debug_scale_gray + 1),
        #               ((simulator["center"] + 1)*debug_scale_gray, (simulator["height"] - (back_pixels - 1))*debug_scale_gray - 1),
        #               gray_color["robot_blue"] if self.sorting_plate_state == sorting_plate_state_dic['BLUE'] else gray_color["robot_red"],
        #               -1)

        return self.frame_gray

    def draw_state(self):
        self.frame = np.zeros((simulator["height"], simulator["width"], 1), np.uint8)
        # Wall
        for obstacle in self.obstacles:
            cx = simulator["center"] + int(round(1.0*obstacle[0]/trans_scale))
            cy = simulator["height"] - back_pixels - int(round(1.0*obstacle[1]/trans_scale))
            if self.check_window_state(cx, cy):
                self.frame[cx][cy] = self._params["Map.data.obstacle"]
        # Red balls
        for r_ball in self.red_balls:
            if self.state_blink == False or random.random() > (0.3 + 0.5*r_ball[1]/3.0/(map_param["height"]/2)):
                if r_ball[1] >= int(ball_blind_slope*(abs(1.0*r_ball[0])-ball_blind_bias)):
                    r_ball_x = r_ball[0]
                    r_ball_y = r_ball[1]
                    if self.state_inaccurate:
                        r_ball_x = r_ball_x + random.random()*map_param["center"]*(0.1*r_ball_x*r_ball_x/map_param["center"]/map_param["center"] - 0.05)
                        r_ball_y = r_ball_y + random.random()*map_param["center"]*(0.1*r_ball_y*r_ball_y/map_param["center"]/map_param["center"] - 0.05)
                    cx = simulator["center"] + int(round(1.0*r_ball_x/trans_scale))
                    cy = simulator["height"] - back_pixels - int(round(1.0*r_ball_y/trans_scale))
                    if self.check_window_state(cx, cy):
                        self.frame[cx][cy] = self._params["Map.data.red_ball"]
        # Blue balls
        for b_ball in self.blue_balls:
            if self.state_blink == False or random.random() > (0.3 + 0.05*b_ball[1]/3.0/(map_param["height"]/2)):
                if b_ball[1] >= int(ball_blind_slope*(abs(1.0*b_ball[0])-ball_blind_bias)):
                    b_ball_x = b_ball[0]
                    b_ball_y = b_ball[1]
                    if self.state_inaccurate:
                        b_ball_x = b_ball_x + random.random()*map_param["center"]*(0.1*b_ball_x*b_ball_x/map_param["center"]/map_param["center"] - 0.05)
                        b_ball_y = b_ball_y + random.random()*map_param["center"]*(0.1*b_ball_y*b_ball_y/map_param["center"]/map_param["center"] - 0.05)
                    cx = simulator["center"] + int(round(1.0*b_ball_x/trans_scale))
                    cy = simulator["height"] - back_pixels - int(round(1.0*b_ball_y/trans_scale))
                    if self.check_window_state(cx, cy):
                        self.frame[cx][cy] = self._params["Map.data.blue_ball"]

        self.frame[simulator["center"]][simulator["height"]-back_pixels] = 255

        self.draw_state_gray()

        return self.frame

    def get_reward(self, action):
        reward = 0
        red_balls_temp = []
        blue_balls_temp = []

        # reward for red ball
        for i, r_ball in enumerate(self.red_balls):
            cx = round(1.0*r_ball[0]/trans_scale)
            cy = round(1.0*r_ball[1]/trans_scale)
            if cy < reward_region_y and cy >= 0 and r_ball[1] >= int(ball_blind_slope*(abs(1.0*r_ball[0])-ball_blind_bias)-2) and (cx in reward_region_x):
                if cx == reward_region_x[1]:
                    reward += 10
                else:
                    reward += 7
                # For sorting plate
                if len(self.red_balls_prev) > 0 and int(round(1.0*self.red_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                    self.sorting_plate_state != sorting_plate_state_dic['RED']:
                    reward = -2
            else:
                red_balls_temp.append(r_ball)

        # reward for blue ball
        for i, b_ball in enumerate(self.blue_balls):
            cx = round(1.0*b_ball[0]/trans_scale)
            cy = round(1.0*b_ball[1]/trans_scale)
            if cy < reward_region_y and cy >=0 and b_ball[1] >= int(ball_blind_slope*(abs(1.0*b_ball[0])-ball_blind_bias)-2) and (cx in reward_region_x):
                if  cx == reward_region_x[1]:
                    reward += 10
                else:
                    reward += 7
                if len(self.blue_balls_prev) > 0 and int(round(1.0*self.blue_balls_prev[i][1]/trans_scale)) < reward_region_y or\
                    self.sorting_plate_state != sorting_plate_state_dic['BLUE']:
                    reward = -2
            else:
                blue_balls_temp.append(b_ball)

        self.red_balls = red_balls_temp
        self.blue_balls = blue_balls_temp

        red_balls_inscreen = []
        blue_balls_inscreen = []
        for r_ball in red_balls_temp:
            if r_ball[1] >= ball_blind_slope * (abs(1.0*r_ball[0]) - ball_blind_bias)\
                and abs(1.0*r_ball[0]) <= map_param["center"] and abs(1.0*r_ball[1]) < map_param["height"]:
                red_balls_inscreen.append(r_ball)
        for b_ball in blue_balls_temp:
            if b_ball[1] >= ball_blind_slope * (abs(1.0*b_ball[0]) - ball_blind_bias)\
                and abs(1.0*b_ball[0]) <= map_param["center"] and abs(1.0*b_ball[1]) < map_param["height"]:
                blue_balls_inscreen.append(b_ball)

        if action in range(self.action_space.size):
            if len(red_balls_inscreen) == 0 and len(blue_balls_inscreen) == 0:
                self.ball_inscreen_flag = self.ball_inscreen_flag + 1
                if action == 8:
                    reward += 0.001
            else:
                self.ball_inscreen_flag = 0

        if (len(red_balls_temp) == 0 and len(blue_balls_temp) == 0) or self.iter > max_iter or self.ball_inscreen_flag >= 10:
            self.done = True

        if self.done:
            self.episode_rewards.append(self.score)
            if self.write_flag:
                self.video.release()
                print ("video saved")

        if action == -1:
            return -1
        else:
            return reward

    def get_reward_wit(self, action):
        reward = 0
        red_balls_temp = []
        blue_balls_temp = []

        # Give reward if we move closer to the nearest ball
        ball_list = np.concatenate([x for x in [self.red_balls, self.blue_balls] if len(x) > 0]).tolist()
        found_flag = False
        if ball_list:
            ball_distances = np.sum(np.asarray(ball_list)**2, axis=1)
            sorted_indices = np.argsort(ball_distances)
            for index in sorted_indices:
                if in_camera_range(ball_list[index]) and no_wall_blocking([0, 0], ball_list[index], self.obstacles):
                    if self.index_prev != -1:
                        nearest_ball_cur = self.red_balls[index] if index < len(self.red_balls) else self.blue_balls[index - len(self.red_balls)]
                        nearest_ball_prev = self.red_balls_prev[self.index_prev] if self.index_prev < len(self.red_balls_prev) else self.blue_balls_prev[self.index_prev - len(self.red_balls_prev)]
                        d_cur = pow(nearest_ball_cur[0], 2) + pow(nearest_ball_cur[1], 2)
                        d_prev = pow(nearest_ball_prev[0], 2) + pow(nearest_ball_prev[1], 2)
                        if d_cur < d_prev and index == self.index_prev:
                            reward += round(0.5*math.sqrt(d_prev - d_cur), 4)
                    self.index_prev = index
                    found_flag = True
                    break
        if not found_flag:
            self.index_prev = -1

        # Give reward if the robot collect the ball
        for i, r_ball in enumerate(self.red_balls):
            # If any ball collectable
            if collectable(r_ball):
                if i == self.index_prev:
                    self.index_prev = -1
                reward += calculate_reward(r_ball)
                # No reward for sorting plate yet
            else:
                red_balls_temp.append(r_ball)

        for i, b_ball in enumerate(self.blue_balls):
            if collectable(b_ball):
                if i + len(self.red_balls) == self.index_prev:
                    self.index_prev = -1
                reward += calculate_reward(b_ball)
                # No reward for sorting plate yet
            else:
                blue_balls_temp.append(b_ball)

        # Remaining balls --------------------------------------------------------------------
        self.red_balls = red_balls_temp  # list
        self.blue_balls = blue_balls_temp

        red_balls_inscreen = []
        blue_balls_inscreen = []
        for r_ball in red_balls_temp:
            if r_ball[1] >= ball_blind_slope*(abs(1.0*r_ball[0]) - ball_blind_bias) and abs(1.0*r_ball[0]) <= map_param["center"] and abs(1.0*r_ball[1]) < map_param["height"]:
                red_balls_inscreen.append(r_ball)
        for b_ball in blue_balls_temp:
            if b_ball[1] >= ball_blind_slope*(abs(1.0*b_ball[0]) - ball_blind_bias) and abs(1.0*b_ball[0]) <= map_param["center"] and abs(1.0*b_ball[1]) < map_param["height"]:
                blue_balls_inscreen.append(b_ball)

        if action in range(self.action_space.size):
            if len(red_balls_inscreen) == 0 and len(blue_balls_inscreen) == 0:
                self.ball_inscreen_flag = self.ball_inscreen_flag + 1
                if action == 8:
                    reward += 0.001
            else:
                self.ball_inscreen_flag = 0

        if (len(red_balls_temp) == 0 and len(blue_balls_temp) == 0) or self.iter > max_iter:  # or self.ball_inscreen_flag >= 10:
            self.done = True
            # Just for debugging to see why it stops
            if len(red_balls_temp) == 0 and len(blue_balls_temp) == 0:
                print('len(red_balls_temp) == 0 and len(blue_balls_temp) == 0')
            if self.iter > max_iter:
                print('self.iter > max_iter')
            if self.ball_inscreen_flag >= 10:
                print('self.ball_inscreen_flag >= 10')

        if self.done:
            self.episode_rewards.append(self.score)
            if self.write_flag:
                self.video.release()
                print ("video saved")

        if action == -1:
            return -1  # Penalize if doesn't move (I think because I bypass in the step funtion if action == -1, we won't reach this point)
        else:
            return reward
    
    def step(self, action):
        if action in range(self.action_space.size):
            self.iter = self.iter + 1

        del_x, del_y, rot = 0, 0, 0

        if action == 0: # forward
            del_x, del_y = 0, -1
        elif action == 1: # forward right
            del_x, del_y = -1, -1
        elif action == 2: # right
            del_x, del_y = -1, 0
        elif action == 3: # backward right
            del_x, del_y = -1, 1
        elif action == 4: # backward
            del_x, del_y = 0, 1
        elif action == 5: # bacward left
            del_x, del_y = 1, 1
        elif action == 6: # left
            del_x, del_y = 1, 0
        elif action == 7: # forward left
            del_x, del_y = 1, -1
        elif action == 8: # rotate left
            rot = -1
        elif action == 9: # rotate right
            rot = 1
        elif action == 10:
            del_x, del_y, self.sorting_plate_state = 0, -1, sorting_plate_state_dic['RED']
        elif action == 11:
            del_x, del_y, self.sorting_plate_state = 0, -1, sorting_plate_state_dic['BLUE']
        else:
            del_x, del_y, rot = 0, 0, 0

        reward = 0
        if action not in [-1]:  # Perform action if action not in this list
            red_balls_temp = []
            blue_balls_temp = []
            obstacles_temp = []

            del_x = del_x  # * 10
            del_y = del_y  # * 10

            if len(self.red_balls) > 0:
                red_balls_temp = np.add(self.red_balls, [del_x,del_y])

            if len(self.blue_balls) > 0:
                blue_balls_temp = np.add(self.blue_balls, [del_x,del_y])

            if len(self.obstacles) > 0:
                obstacles_temp = np.add(self.obstacles, [del_x,del_y])

            if action == 8 or action == 9:
                points = np.concatenate([x for x in [red_balls_temp, blue_balls_temp, obstacles_temp] if len(x) > 0])
                if points.size > 0:
                    points = points.reshape(-1,2)
                    theta = rot_scale*rot*np.pi/180
                    theta_0 = np.arctan2(points.T[1], points.T[0])

                    ball_dist = np.linalg.norm(points, axis=1)
                    rot_delta_unit_x = np.subtract(np.cos(theta_0), np.cos(np.add(theta_0,theta)))
                    rot_delta_unit_y = np.subtract(np.sin(theta_0), np.sin(np.add(theta_0,theta)))
                    rot_delta_unit = np.concatenate((rot_delta_unit_x.reshape(-1,1), rot_delta_unit_y.reshape(-1,1)), axis=1)
                    ball_dist = np.concatenate((ball_dist.reshape(-1,1), ball_dist.reshape(-1,1)), axis=1)
                    rot_delta = np.multiply(ball_dist, rot_delta_unit)
                    points = np.subtract(points, rot_delta)
                    red_balls_temp = points[0:len(self.red_balls)]  # This is numpy array. Be careful
                    blue_balls_temp = points[len(self.red_balls):len(self.red_balls)+len(self.blue_balls)]
                    obstacles_temp = points[len(self.red_balls)+len(self.blue_balls):]

            enable_move = True
            for obstacle in obstacles_temp:
                if abs(obstacle[0]) < R_X and abs(obstacle[1]) < R_Y:  # Here, parameter is in map scale
                # if abs(1.0*obstacle[0]) < 4.0*trans_scale/3 and abs(1.0*obstacle[1]) < 8.0*trans_scale/3:
                    enable_move = False

            if enable_move:
                self.red_balls = red_balls_temp  # This is numpy array. Be careful
                self.blue_balls = blue_balls_temp
                reward = self.get_reward_wit(action)
                self.obstacles = obstacles_temp
                self.draw_state()
                self.red_balls_prev = self.red_balls
                self.blue_balls_prev = self.blue_balls
            else:
                # print('Cannot move')  # Just for debugging
                # reward = self.get_reward_wit(-1)  # Can we just bypass this if cannot move? Because nothing will change?
                reward = -2
        else:
            reward = -1  # Doesn't do anything

        self.score = self.score + reward
        self.current_reward = reward  # Add by myself

        if self.write_flag:
            frame_debug = self.draw_debug_frame(self.frame)
            self.video.write(frame_debug)

        if self.debug_flag:
            frame_debug = self.draw_debug_frame(self.frame)
            # cv2.imshow("frame_debug", frame_debug)
            cv2.imshow("frame_debug_gray", self.frame_gray)
            cv2.imshow("frame_wit", self.draw_environment())  # Add by myself
            cv2.waitKey(100)

        return self.frame_gray, reward, self.done

    def get_total_steps(self):
        return self.iter

    def get_episode_rewards(self):
        return self.episode_rewards

    def action_space_sample(self):
        index = int(1.0*random.random()*self.action_space.size)
        return self.action_space[index]
    

# Added functions ##############################################################################
def closest_point(point, point_list):
    """
    Return index of a point in the list that is closest to the given point, -1 if no such point
    """
    if len(point_list):
        points = np.asarray(point_list)
        distances = np.sum((points - point)**2, axis=1)
        index = np.argmin(distances)
        return index
    else:
        return -1


def distance_from_point(point, point_list):
    """
    Calculate all distances from point to each point in the point_list
    """
    if len(point_list):
        return np.sum((np.asarray(point_list) - point)**2, axis=1)
    else:
        return []


def get_nearest_seeable_ball_index(point, point_list, obstacle_list):
    """
    Get index of the nearest seeable ball in the point_list, -1 if no such ball
    """
    if point_list:
        ball_distances = np.sum((np.asarray(point_list) - point)**2, axis=1)
        sorted_indices = np.argsort(ball_distances)
        for index in sorted_indices:
            if in_camera_range(point_list[index]) and no_wall_blocking([0, 0], point_list[index], obstacle_list):
                return index
        return -1
    else:
        return -1


def in_camera_range(pt):
    """
    Check if pt is in the camera range

    Input: [x,y] in map scale
    """
    if (pt[1] > 0) and (pt[1] >  ball_blind_slope*(abs(pt[0]) - ball_blind_bias_map)):
        return True
    else:
        return False


def collectable(pt):
    """
    Check if the ball can be collected

    Input: [x,y] in map scale
    """
    if (R_Y <= pt[1] and pt[1] < R_Y + COLLECT_Y) and (pt[1] >  ball_blind_slope*(abs(pt[0]) - ball_blind_bias_map)) and abs(pt[0]) <= COLLECT_X:
        return True
    else:
        return False


def calculate_reward(pt):
    """
    Calculate reward for a ball at pt

    Input: [x,y] in map scale
    """
    d = abs(pt[0])
    if d < COLLECT_X/3:
        return 10
    else:
        return 7


def no_wall_blocking(pt1, pt2, obstacles_list):
    """
    Check if there is a wall blocking a path from pt1 to pt2

    Input: [x,y] in map scale
    
    Note: Valid only when the ball in front of the robot
    """
    wall_integer = np.int_(np.round(obstacles_list)).tolist()
    loop_x = True if pt2[0]-pt1[0] >= pt2[1]-pt1[1] else False

    if loop_x:  # Check above and below
        slope = (pt2[1] - pt1[1])/(pt2[0] - pt1[0])
        for x in range(int(round(pt1[0] + R_X)), int(round(pt2[0]))):
            y = int(round(slope*x))
            if ([x, y-1] in wall_integer) or ([x, y+1] in wall_integer):
                return False
    else:  # Check left and right
        slope = (pt2[0] - pt1[0])/(pt2[1] - pt1[1])
        for y in range(int(round(pt1[1] + R_Y)), int(round(pt2[1]))):
            x = int(round(slope*y))
            if ([x-1, y] in wall_integer) or ([x+1, y] in wall_integer):
                return False
    return True

###############################################################################################


if __name__ == '__main__':
    tk = Task(debug_flag=True, test_flag=False, state_blink=False, state_inaccurate=False)
    tk.reset()

    action = -1
    while(1):
        tk.step(action)
        key = cv2.waitKey(300)&0xFF
        action = -1
        if key == ord('q') or tk.done == True:
            break
        
        elif key == ord('e'):  # forward
            action = 0
        elif key == ord('r'):  # forward right
            action = 1
        elif key == ord('w'):  # forward left
            action = 7
        
        elif key == ord('f'):  # right
            action = 2
        elif key == ord('s'):  # left
            action = 6
        
        
        elif key == ord('d'):  # backward
            action = 4
        elif key == ord('v'):  # backward right
            action = 3
        elif key == ord('x'):  # bacward left
            action = 5
        
        
        elif key == ord('a'):  # Rotate left
            action = 8
        elif key == ord('g'):  # Rotate right
            action = 9
        
        elif key == ord('1'):
            action = 10
        elif key == ord('2'):
            action = 11

    print("shutdown")
    # cv2.destroyAllWindows()
