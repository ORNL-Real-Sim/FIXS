import win32com.client
import subprocess
import time

# 1) Start Vissim if it’s not already running
subprocess.Popen([
    r"C:\Program Files\PTV Vision\PTV Vissim 2022\Exe\Vissim220.exe",
    "-Automation"
])
# give it a few seconds to come up
time.sleep(3)

# 2) Attach to it
vissim = win32com.client.DispatchEx("Vissim.Vissim" )
print("Connected to Vissim COM on port 5000")
