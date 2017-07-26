# Patrolling-robot

requisite: Compile assistive-robotics-repo (you can find the information in ARG bibucket)
           install pcl

# Compile the repo
  laptop  $cd code 
  laptop  $git clone https://github.com/YongShanSue/Patrolling-robot.git
  laptop  $cd Patrolling-robot
  laptop  $ls
  You will find some packages
  laptop  $cd kinect-lcm
  laptop  $make clean
  laptop  $make

# repeate this method and compile all the package
# Now you will have some command to use: rv-kinect-frame-pcl-kobe-demo...........
# If you want to know what command to use, please see CMakeList.txt in every package.

# Find the segmentlists
  Download one of the logdata in the following address:
  https://drive.google.com/drive/u/1/folders/0B3Bm1CtYcrCGRHZMY0NGMUl4MlU

# Play the logdata
  open a terminal
  
  laptop  $lcm-logplayer-gui $(DataName)
# Visualize the log data
  open a terminal
  
  laptop  $kinect-viewer
# Run the code to find the groundline
  open a terminal
  
  laptop  $rv-kinect-frame-pcl-kobe-demo2
# Visualize the outcome
  open a terminal
  
  laptop  $bot-lcmgl-viewer
