import os
import numpy as np
import torch
import torch.nn as nn

import simulator
import cv2


frame_history_len=4

dtype = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor
env = simulator.Task(debug_flag=True, test_flag=False, state_blink=False, state_inaccurate=False)


if len(env.observation_space.shape) == 1:
    # This means we are running on low-dimensional observations (e.g. RAM)
    input_arg = env.observation_space.shape[0]
else:
    img_h, img_w, img_c = env.observation_space.shape
    input_arg = frame_history_len * img_c

test_model = torch.load('DQN_net1122 - Copy.pt')
print('DQN_net1122 - Copy')

input_image = np.zeros((input_arg,img_w,img_h), np.uint8)


obs = env.reset()
for i in range(4):
    input_image[i]=obs[0]

num_step=0
done=False

while(1):        

    if num_step>3:
        input_image[0]=input_image[1]
        input_image[1]=input_image[2]
        input_image[2]=input_image[3]
        input_image[3]=obs[:,:,0]

    elif num_step==1:
        for i in range(4):
             input_image[i]=obs[:,:,0]
    elif num_step==2:
        input_image[2]=obs[:,:,0]
        input_image[3]=obs[:,:,0]
    elif num_step==3:
        input_image[1]=input_image[2]
        input_image[2]=obs[:,:,0]
        input_image[3]=obs[:,:,0]

        
    image = torch.from_numpy(input_image).type(dtype).unsqueeze(0)/255.0
    action =torch.IntTensor([[test_model(image).data.max(1)[1].cpu()]])[0,0]
    print(test_model(image).data, num_step)

    obs, reward, done = env.step(action)

    num_step=num_step+1
    if done:
        obs = env.reset()
        for i in range(4):
            input_image[i]=obs[:,:,0]
        num_step=0

