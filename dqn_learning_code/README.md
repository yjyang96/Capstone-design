# Copy from

1. This file is copied/apdated from https://github.com/berkeleydeeprlcourse/homework/tree/master/hw3
2. This file is copied/apdated from https://github.com/leeJaeDuck/dqn_learning_code

# Assignment 5

1. 팀 소개
BAM조
오준호 교수님
20140344 양예준/20140870 이보미/20150629 이해우

2. 수정한 파일 명
simulator.py
map_gen.yaml
blockdiagram/simulator.pdf
data/video/tt.video.001717.0.mp4

3. 리뷰한 파일 명
dqn_learn.py
learning review.txt

# 환경
window10
python3 64bit
opencv, pytorch, numpy, pyyaml

# Usage

To train a model:

```
$ python main.py

```

To test simualation:

```
$ python simulator.py

```

To test network with simulator:

```
$ python Network_test.py

```


The model is defined in `dqn_model.py`

The algorithm is defined in `dqn_learn.py`

The running script and hyper-parameters are defined in `main.py`

related paper link https://www.nature.com/articles/nature14236.pdf

-------------------------------------------------------------------
if you want to use tensorboard in pythorch(monitoring DNN)

1. install  tensorboardX
```
pip install tensorboardX
```
2. isntall tensorflow
```
pip install tensorflow
```
3. launch tensorboard at your directory
```
cd dqn_learning_code
tensorboard --logdir runs
```
4. copy the address and paste the address at webbrowser

ex) http://humanlab-Z370-AORUS-Gaming-3:6006