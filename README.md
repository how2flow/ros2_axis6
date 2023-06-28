# axis6

An ros2 package that uses an odroid to control the six-axis robot arm.

### requirements

#### hardware
- [robot frame](https://www.aliexpress.com/item/32858303546.html?pdp_npi=2%40dis%21KRW%21%E2%82%A9%2042%2C155%21%E2%82%A9%2040%2C463%21%21%21%21%21%402103010b16824857878262784e96d2%2165326785008%21btf&_t=pvid%3Ad28f4b46-8eff-49f3-b543-134dce62b94c&afTraceInfo=32858303546__pc__pcBridgePPC__xxxxxx__1682485788&spm=a2g0o.ppclist.product.mainProduct&gatewayAdapt=glo2kor)
- [MG996R](https://www.aliexpress.com/item/1005005200022250.html?spm=a2g0o.productlist.main.1.52a93d167VwLLG&algo_pvid=a2ee8dde-5400-4bbc-8e5b-eff5a24176d7&aem_p4p_detail=202304252211317561106546918600023300123&algo_exp_id=a2ee8dde-5400-4bbc-8e5b-eff5a24176d7-0&pdp_npi=3%40dis%21KRW%214979.0%214731.0%21%21%21%21%21%40211bc2a016824858919401497d075e%2112000032121202928%21sea%21KR%210&curPageLogUid=I8jXdGUt85Q1&ad_pvid=202304252211317561106546918600023300123_1&ad_pvid=202304252211317561106546918600023300123_1) x 6
- pca9685: [How to Use](https://www.how2flow.net/posts/pca9685)

#### develop env (operator)

- kernel 5.15.y
- ros2-foxy
- libgpiod
- adafruit-circuitpython-pca9685
- odroid-wiringpi

### fixups

Install python3 packages
```
$ sudo apt-get update
$ sudo apt-get install python3 python3-dev python3-pip
```
<br>

Install pip packages
```
$ sudo python3 -m pip install --upgrade pip
$ sudo python3 -m pip install adafruit-circuitpython-pca9685 \
  adafruit-python-shell click wheel \
  Adafruit-Blinka==8.18.1 \
  adafruit-circuitpython-register \
  adafruit-circuitpython-busdevice
```
<br>

libgpiod preinstall
```
$ wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/libgpiod.py
$ sudo python3 libgpiod.py # ignore errors
```
<br>

Install libgpiod
```
$ git clone https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
$ cd libgpiod
$ git checkout v1.4.2 -b v1.4.2
$ sudo ./autogen.sh --enable-tools=yes --prefix=/usr/local/ --enable-bindings-python CFLAGS="-I/$include_path"
$ sudo make
$ sudo ldconfig
$ sudo cp bindings/python/.libs/gpiod.* /usr/local/lib/python3.?/dist-packages
```
<br>

### Usage

How to Build
```
$ sudo groupadd gpiod --gid {group_id}
$ sudo usermod -aG gpiod $(whoami)
$ git clone --recurse-submodules https://github.com/how2flow/ros2_axis6.git
$ cd ros2_axis6
$ ./preinstall.sh
$ reboot
```
```
$ mkdir -p robot_ws/src
$ ln -s ~/ros2_axis6 robot_ws/src
$ ln -s ~/ros2_axis6/how2flow_interfaces ~/robot_ws/src
$ cd ~/robot_ws && colcon build --symlink-install
```
<br>

How to Run?

commander
```
$ ros2 run axis6 commander
```

operator (odroid)
```
$ ros2 run axis6 operator
```
<br>

View and Contorl prototype in rviz<br>

requirements: ros-foxy-joint-state-publisher-gui
```
$ ros2 launch display.prototype.py
```
![prototpye-rviz](images/rviz.png)
