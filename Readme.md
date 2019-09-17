
#### CE5045-嵌入式系統設計 Embedded System Design-*-1081
---
# 課程內會使用到的實驗板以及相關軟體

## 開發板使用 ST 的 NUCLEO-H743ZI
https://www.st.com/en/evaluation-tools/nucleo-h743zi.html
![](https://i.imgur.com/o0u3mks.jpg)



 


---

## 課程會用到的軟體。請同學自行安裝，如有疑問請洽助教。




* ### 安裝STM32CubeMX 5.3.0 
(需要先裝JVM，使用macOS/Ubuntu同學請先往下看)
https://java.com/zh_TW/download/

https://drive.google.com/file/d/1f_5kfOdgtND2GsflpIMc3FlHuXv3aR0m/view?usp=sharing

 

* ### 安裝 HAL Lib 函式庫
在 STM32CubeMX 起始畫面的工具列 [Help] > [Manage embedded software packages] 內
找到並安裝 :
 STM32Cube MCU Package for STM32H7 Series 1.5.0
![](https://i.imgur.com/HYy1hlP.png)

![](https://i.imgur.com/ZELx7gW.png)

 

 

* ### 安裝 StLink驅動程式
https://drive.google.com/file/d/1puMhmMw7YZdF2WQvz1EEGkciX4UjGX0m/view?usp=sharing



* ### 安裝Python 3
https://www.python.org/downloads/

 

* ### 安裝Git
https://git-scm.com/

 

* ### 安裝 VSCode
https://code.visualstudio.com/
![](https://i.imgur.com/ZtSM2ww.png)

 

* ### 在 VSCode 內安裝以下 Extension (視窗左下角)：
C/C++

Cortex-Debug

Python

GitLens



## <font color=blue>使用 Windows 的同學</font>


* ### 安裝/下載 putty (或慣用之Serial Terminal)
https://www.putty.org/

 

 

 

## <font color=green>使用 macOS 的同學</font>

 

* ### 若無JVM，可用以下指令安裝
 

```
$ brew cask install java

安裝完成後可用指令查看版本
$ java --version
``` 


 

* ### 安裝 arm gcc
 

```
$ brew tap ArmMbed/homebrew-formulae
$ brew install arm-none-eabi-gcc

安裝完成後可用指令查看版本
$ arm-none-eabi-gcc --version
```

 

 

 

* ### 安裝 openocd
 

```
$ brew install open-ocd
```

 
* ### 安裝熟悉之 Serial Terminal 或熟悉 screen 指令之使用
https://learn.adafruit.com/welcome-to-circuitpython/advanced-serial-console-on-mac-and-linux

 
 

## <font color=orange>使用 Ubuntu 的同學</font>

 

 

* ### 安裝JVM
```
$ sudo apt-get install default-jdk default-jre
```

 

* ### 安裝 git
 

```
$ sudo apt-get install git
```

 

* ### 安裝 arm toolchain
```
$ sudo apt-get install gcc-arm-none-eabi gdb-multiarch

$ sudo ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb
```

 
* ### 安裝熟悉之 Serial Terminal 或熟悉 screen 指令之使用
https://learn.adafruit.com/welcome-to-circuitpython/advanced-serial-console-on-mac-and-linux
 

* ### 改變 STLink USB 設備權限 (需要重開機)
在此檔內加四下面四行"/etc/udev/rules.d/98-stlink.rules" :
> SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374b", MODE="666"
> SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="666"
> SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3744", MODE="666"
> SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="666"




---


# 安裝流程 Q & A
<font color=red>Q：我是macOS，我已經裝好java，但在安裝STM32CubeMX時雙擊卻一直失敗，要怎麼辦？</font>
<font color=green>A：請試試以下的步驟：</font>
> 1. 兩指單擊 SetupSTM32CubeMX-5.3.0 
> ![](https://i.imgur.com/kADKgK3.png)
> 
> 2. 在Contents/MacOs 資料夾上兩指單擊，選擇[新增位於檔案夾位置的終端機視窗]
> ![](https://i.imgur.com/gxuTwJT.png)
> 
> 3. 在終端機視窗上輸入以下指令
> ```
> $ sudo ./SetupSTM32CubeMX-5_3_0_macos
> ```
> ![](https://i.imgur.com/1q5eRQw.png)
> 
> 4. 成功叫出安裝畫面後點Next進行後續安裝
> ![](https://i.imgur.com/44BgMUW.png)



