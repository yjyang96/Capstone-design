"""
    This file is copied/apdated from https://github.com/berkeleydeeprlcourse/homework/tree/master/hw3
"""
import sys
import numpy as np
from collections import namedtuple
from itertools import count
import random
import os
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.autograd as autograd
# ReplayBuffer는 값을 받아 순서대로 쌓아주고, 필요한 값을 원할 때 불러올 수 있게 해줌
from utils.replay_buffer import ReplayBuffer
# 그림을 그릴 때 필요한 tensorboardX에서 SummaryWriter import
from tensorboardX import SummaryWriter

# Cuda가 사용 가능한지 체크하고 그에 따라 datatype 저장
USE_CUDA = torch.cuda.is_available()
dtype = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor

# Cuda관련 settings
class Variable(autograd.Variable):
    def __init__(self, data, *args, **kwargs):
        if USE_CUDA:
            data = data.cuda()
        super(Variable, self).__init__(data, *args, **kwargs)

"""
    OptimizerSpec containing following attributes
        constructor: The optimizer constructor ex: RMSprop
        kwargs: {Dict} arguments for constructing optimizer
"""

# Optimizer Specification. Optimizer로 쓰일 constructor와 kwargs, 즉 arguments들이 tuple형태로 저장
OptimizerSpec = namedtuple("OptimizerSpec", ["constructor", "kwargs"])

# 함수 DQN learning정의

## DQN learning의 input parameters는 다음과 같다
### env: enviornment, 즉 training하게 될 이미지
### q_func: Q function을 계산할 때 사용할 모델
### optimizer_spec: Optimizer 관련 specifications
### exploration: exploration epsilon greedy schedule 설정
### stopping_criterion: learning을 멈출 조건
### replay_buffer_size: ReplayBuffer안에 저장할 수 있는 메모리 크기
### batch_size: Batch 사이즈
### gamma: discount reward gamma 설정
### learning_starts: learning을 시작하기 전까지 필요한
### learning_freq: Q 업데이트 주기
### frame_history_len: 저장하고 사용할 이전 화면의 갯수
### target_update_freq:타겟 업데이트 주기

def dqn_learing(
    env,
    q_func,
    optimizer_spec,
    exploration,
    stopping_criterion=None,
    replay_buffer_size=1000000,
    batch_size=32,
    gamma=0.99,
    learning_starts=50000,
    learning_freq=4,
    frame_history_len=4,
    target_update_freq=10000
    ):

    """Run Deep Q-learning algorithm.

    You can specify your own convnet using q_func.

    All schedules are w.r.t. total number of steps taken in the environment.

    Parameters
    ----------
    env: gym.Env
        gym environment to train on.
    q_func: function
        Model to use for computing the q function. It should accept the
        following named arguments:
            input_channel: int
                number of channel of input.
            num_actions: int
                number of actions
    optimizer_spec: OptimizerSpec
        Specifying the constructor and kwargs, as well as learning rate schedule
        for the optimizer
    exploration: Schedule (defined in utils.schedule)
        schedule for probability of chosing random action.
    stopping_criterion: (env) -> bool
        should return true when it's ok for the RL algorithm to stop.
        takes in env and the number of steps executed so far.
    replay_buffer_size: int
        How many memories to store in the replay buffer.
    batch_size: int
        How many transitions to sample each time experience is replayed.
    gamma: float
        Discount Factor
    learning_starts: int
        After how many environment steps to start replaying experiences
    learning_freq: int
        How many steps of environment to take between every experience replay
    frame_history_len: int
        How many past frames to include as input to the model.
    target_update_freq: int
        How many experience replay rounds (not steps!) to perform between
        each update to the target Q network
    """

    ############################
    # BUILD MODEL / 모델 만들기 #
    ############################

    # Observation에 따라 Q function에 들어갈 input_arg를 설정
    if len(env.observation_space.shape) == 1:
        # This means we are running on low-dimensional observations (e.g. RAM)
        input_arg = env.observation_space.shape[0]
    else:
        img_h, img_w, img_c = env.observation_space.shape
        input_arg = frame_history_len * img_c

    # Simulator에서의 action의 갯수 받아옴
    num_actions = env.action_space.size


    # Construct an epilson greedy policy with given exploration schedule
    # Epsilon greedy 함수 설정
    ## random으로 뽑은 sample값과 dqn learning의 exploration schedule과 비교, 결과에 맞게 epsilon greedy action policy를 줌
    def select_epilson_greedy_action(model, obs, t):
        sample = random.random()
        eps_threshold = exploration.value(t)
        #
        if sample > eps_threshold:
            obs = torch.from_numpy(obs).type(dtype).unsqueeze(0) / 255.0
            # Use volatile = True if variable is only used in inference mode, i.e. don’t save the history
            return torch.IntTensor([[model(Variable(obs)).data.max(1)[1].cpu()]])
        else:
            return torch.IntTensor([[random.randrange(num_actions)]])


    # Initialize target q function and q function
    # Q function과 target Q function을 정의
    Q = q_func(input_arg, num_actions).type(dtype)
    target_Q = q_func(input_arg, num_actions).type(dtype)

    # Construct Q network optimizer function
    # Optimizer_spec을 이용하여 optimizer function을 만듦
    optimizer = optimizer_spec.constructor(Q.parameters(), **optimizer_spec.kwargs)

    # Construct the replay buffer
    # ReplayBuffer을 이용해 replay_buffer 생성
    replay_buffer = ReplayBuffer(replay_buffer_size, frame_history_len)

    ###############
    # RUN ENV     #
    ###############
    num_param_updates = 0
    mean_episode_reward = -float('nan')
    best_mean_episode_reward = -float('inf')
    last_obs = env.reset()
    LOG_EVERY_N_STEPS = 10000

    # TensorboardX를 모니터함
    writer = SummaryWriter()

    # t가 0부터 loop이 돌 때 마다 하나씩 커짐. 몇 번의 iteration이 실행되었는지 확인 가능
    for t in count():
        ### Step the env and store the transition
        # Store lastest observation in replay memory and last_idx can be used to store action, reward, done

        # 가장 최근의 결과 이미지가 replay_buffer에 저장되고 그때의 action, reward, 그리고 끝남의 여부가 last_idx에 저장됨
        last_idx = replay_buffer.store_frame(last_obs)

        # encode_recent_observation will take the latest observation
        # that you pushed into the buffer and compute the corresponding
        # input that should be given to a Q network by appending some
        # previous frames.

        #replay_buffer에 저장된 buffer중 가장 최근 것을 불러 직전의 frame들과 비교, Q network에 들어갈 input을 계산
        recent_observations = replay_buffer.encode_recent_observation()

        # Choose random action if not yet start learning
        # t가 learning_starts보다 크다면, 즉 충분한 iteration이 진행되었다면 action을 random값이 아닌 learning에 의한 값으로 받음
        if t > learning_starts:
            action = select_epilson_greedy_action(Q, recent_observations, t)[0, 0]
        else:
            action = random.randrange(num_actions)

        # Advance one step
        # action을 취하고 그에 따른 결과 이미지 (obs), 보상 (reward), 끝남 여부 (done)을 저장, replay_buffer에도 넣어줌
        obs, reward, done = env.step(action)
        replay_buffer.store_effect(last_idx, action, reward, done)

        # Resets the environment when reaching an episode boundary.
        # 끝이 났다면 env, 즉 학습 환경도 다시 리셋함
        if done:
            obs = env.reset()
        last_obs = obs

        ### Perform experience replay and train the network.
        # Note that this is only done if the replay buffer contains enough samples
        # for us to learn something useful -- until then, the model will not be
        # initialized and random actions should be taken

        ## 충분한 iteration으로 t가 learning_starts보다 크고,
        ## learning_freq의 주기와 맞고,
        ## buffer의 사이즈가 batch 사이즈와 비교해 충분 할 때, learning이 시작됨
        if (t > learning_starts and
                t % learning_freq == 0 and
                replay_buffer.can_sample(batch_size)):

            # Use the replay buffer to sample a batch of transitions
            # Note: done_mask[i] is 1 if the next state corresponds to the end of an episode,
            # in which case there is no Q-value at the next state; at the end of an
            # episode, only the current state reward contributes to the target

            #replay_buffer에서 batch size에 맞는 데이터 양을 불러온다
            obs_batch, act_batch, rew_batch, next_obs_batch, done_mask = replay_buffer.sample(batch_size)

            # Convert numpy nd_array to torch variables for calculation

            # model의 input에 맞게 numpy array에서 torch로 변환
            obs_batch = Variable(torch.from_numpy(obs_batch).type(dtype) / 255.0)
            act_batch = Variable(torch.from_numpy(act_batch).long())
            rew_batch = Variable(torch.from_numpy(rew_batch))
            next_obs_batch = Variable(torch.from_numpy(next_obs_batch).type(dtype) / 255.0)
            not_done_mask = Variable(torch.from_numpy(1 - done_mask)).type(dtype)

            if USE_CUDA:
                act_batch = act_batch.cuda()
                rew_batch = rew_batch.cuda()

            # Compute current Q value, q_func takes only state and output value for every state-action pair
            # We choose Q based on action taken.
            # 현재의 Q value를 계산
            current_Q_values = Q(obs_batch).gather(1, act_batch.unsqueeze(1))

            # Compute next Q value based on which action gives max Q values
            # Detach variable from the current graph since we don't want gradients for next Q to propagated
            # 어떤 action이 max Q value를 주는지에 따라 다음 Q value를 설정
            next_max_q = target_Q(next_obs_batch).detach().max(1)[0]
            next_Q_values = not_done_mask * next_max_q
            # Compute the target of the current Q values
            # 현재의 targer Q value를 optimize와 backward를 이용하여 계산
            target_Q_values = rew_batch + (gamma * next_Q_values)
            loss = F.smooth_l1_loss(current_Q_values, target_Q_values.unsqueeze(1))
            optimizer.zero_grad()
            loss.backward()

            # Perfom the update
            # 업데이트 후 업데이트 횟수도 업데이트
            optimizer.step()
            num_param_updates += 1

            # Periodically update the target network by Q network to target Q network
            # targer 업데이트 주기에 맞을때 마다 target network를 업데이트
            if num_param_updates % target_update_freq == 0:
                target_Q.load_state_dict(Q.state_dict())


        # ### 4. Log progress and keep track of statistics
        # episode reward 출력, 100번이 넘어가면 최고평균값도 평균값과 함께 출력
        episode_rewards = env.get_episode_rewards()
        if len(episode_rewards) > 0:
            mean_episode_reward = np.mean(episode_rewards[-100:])
        if len(episode_rewards) > 100:
            best_mean_episode_reward = max(best_mean_episode_reward, mean_episode_reward)

        # Tensorboard에 저장
        if len(episode_rewards) > 0:
            writer.add_scalar('data/DQN/score', episode_rewards[-1], len(episode_rewards))
            writer.add_scalar('data/DQN/mean_score', mean_episode_reward, len(episode_rewards))
            if len(episode_rewards) > 100:
                writer.add_scalar('data/DQN/best_mean_score', best_mean_episode_reward, len(episode_rewards))

        # learning된 내용 출력
        if t % LOG_EVERY_N_STEPS == 0 and t > learning_starts:
            print("Timestep %d" % (t,))
            print("mean reward (100 episodes) %f" % mean_episode_reward)
            print("best mean reward %f" % best_mean_episode_reward)
            print("episodes %d" % len(episode_rewards))
            print("exploration %f" % exploration.value(t))
            sys.stdout.flush()
            torch.save(Q, 'DQN_net1029.pt')
            ## file 이름 바꾸기

    writer.close()
