# Camera Control

-----

* [Webcam Capture](http://webcam-capture.sarxos.pl/): Generic Webcam Java API
* [Set up a Webcam with Linux](http://www.linuxintro.org/wiki/Set_up_a_Webcam_with_Linux)
* [Accessing the Video Device](https://www.tldp.org/HOWTO/Webcam-HOWTO/dev-intro.html)
  Create Video Device ( /dev/video1 )
  ```
  sudo mknod /dev/video1 c 81 1
  sudo chmod 666 /dev/video1
  sudo chgrp video /dev/video1
  ```

## Drivers

* Video4Linux
  - V4L for short, is a collection of device drivers and an API for supporting realtime video capture on Linux systems.
  - v4l2-ctl

* Firewire
  - [The IEEE1394/USB3 Digital Camera List](https://damien.douxchamps.net/ieee1394/cameras/)

* UVC (USB Video Class)

## Viewer/Controller
- [Webcam Test (Online)](https://webcamtests.com/)
- [Turn Camera On (Online)](https://turncameraon.com/)
- [Coriander](https://damien.douxchamps.net/ieee1394/coriander/) is the Linux graphical user interface (GUI) for controlling a Digital Camera through the IEEE1394 bus (aka FireWire, or iLink).
- luvcview: Sdl video Usb Video Class grabber  
- [GTK+ UVC Viewer](http://guvcview.sourceforge.net/index.html)
  * guvcview
- cheese: Take photos and videos with your webcam, with fun graphical effects
- [Motion](https://motion-project.github.io/): a highly configurable program that monitors video signals from many types of cameras
  * [如何借助Motion操控Linux监控摄像头](http://blog.sae.sina.com.cn/archives/4902)
  * [树莓派+motion安装摄像头实现远程监控](http://shumeipai.nxez.com/2016/09/01/raspberry-pi-motion-cameras-for-remote-monitoring.html)
- [mjpg-streamer](https://github.com/jacksonliam/mjpg-streamer)
- [Video Capture Tools (Windows)](http://noeld.com/programs.asp?cat=video)

### Others
* [Linux and webcams](http://krustev.net/w/articles/Linux_and_webcams/)
* [How to Operate Your Spycams with ZoneMinder on Linux (part 1)](https://www.linux.com/learn/how-operate-your-spycams-zoneminder-linux-part-1)
