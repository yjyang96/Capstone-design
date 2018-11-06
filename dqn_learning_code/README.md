# Usage

To train a model:

```
$ python3 main.py or python main.py

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