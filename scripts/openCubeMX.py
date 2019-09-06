import subprocess
import os
import platform

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
        subprocess.call( ["/usr/bin/open", "-W", "-n", "-a", "/Applications/STMicroelectronics/STM32CubeMX.app", "--args", iocFile] )
    elif currPlatform == "Linux":
        print("not done yet")
    elif currPlatform == "Windows":
        print("not done yet")
