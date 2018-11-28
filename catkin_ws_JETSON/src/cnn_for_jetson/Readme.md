
###  Modify your code

before you run testimage_publisher, Please change image directory


image_1 = cv::imread("write down youre image directory!!!!.325.jpg",CV_LOAD_IMAGE_COLOR);

image_2 = cv::imread("/"write down youre image directory!!!!/dog.232.jpg",CV_LOAD_IMAGE_COLOR);



### Before build the source on jetson, Please install dirver and library.

----------------------------------------------------------------------------------------------------
commend below lines 

```
sudo apt-get -y update
sudo apt-get -y install --no-install-recommends \
                    libncurses5-dev \
                    libudev-dev \
                    libeigen3-dev \
                    git \
                    tmux \
                    curl \
                    wget \
                    htop \
                    python\
                    python-pip \
                    python-tk \
                    python-setuptools\
                    python3\
                    python3-pip \
                    python3-tk \
                    python3-setuptools
```
install Rospackage in python.
```
pip install -U numpy \
                    rospkg \
                    catkin-pkg \
                    
pip3 install -U numpy \
                    rospkg \
                    catkin-pkg \

```
install torchvision(using source)
```
git clone https://github.com/pytorch/vision
cd vision
python3 setup.py install

```

### If you have a problem with below error.(for running "catdog_cnn_network.py")

ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type

- These errors are caused by conflicts between Ross's OpenCV(python) and Ubuntu's OpenCV(python).
So you can solve this problem by disabling ROS;s openCV(python).

```
cd /opt/ros/kinetic/lib/python2.7/dist-packages
sudo mv cv2.so cv2.so.copy
```


