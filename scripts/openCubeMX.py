import subprocess
import os
import os.path
import platform

#macOs:
appPathMac = '/Applications/STMicroelectronics/STM32CubeMX.app'
appPathWin64 = 'C:/Program Files (x86)/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe'
appPathWin32 = 'C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe'
appPathUbuntu = os.path.expanduser("~") +'/STM32CubeMX/STM32CubeMX'


os.chdir("../../")
iocFile = ""
for r, d, f in os.walk("./"):
    for file in f:
        if '.ioc' in file:
            iocFile = file

if iocFile != "" :
    iocFile = os.getcwd() + "/" + iocFile
    currPlatform = platform.system()
    currRelease = platform.release()
    print("current Platform is", currPlatform)
    print("current Release is", currRelease)
    print("opening file :",iocFile)
    if currPlatform == "Darwin":
        if (os.path.exists(appPathMac)):
            comm = ["/usr/bin/open", "-W", "-n", "-a", appPathMac, "--args", iocFile]
            subprocess.run(comm)
        else:
            print("Can't find STM32CubeMX.app with default install path:",appPathMac)
    elif currPlatform == "Linux":
        if (os.path.exists(appPathUbuntu)):
            comm = [appPathUbuntu, iocFile]
            subprocess.run(comm,shell=True)
        else:
            print("Can't find STM32CubeMX with default install path : ",appPathUbuntu)
    elif currPlatform == "Windows":
        appPathWin64 = 'C:/Program Files (x86)/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe'
        appPathWin32 = 'C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe'
        if (os.path.exists(appPathWin64)):
            print("find ",appPathWin64)
            comm = [appPathWin64, iocFile]
            subprocess.run(comm, shell=True)
        elif (os.path.exists(appPathWin32)):
            print("find ",appPathWin32)
            comm = [appPathWin32, iocFile]
            subprocess.run(comm, shell=True)
        else:
            print("Can't find STM32CubeMX.exe with default install path : ",appPathWin64," or ",appPathWin32)
