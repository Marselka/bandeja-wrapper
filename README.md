## samsung-avatar-dataset-sync


Uses ROS, **OpenCamera Sensors** remote control API and Twist-n-Sync synchronization approach to record synchronized data from heterogenious hardware-software system.

To start recording run

`master.py <SMARTPHONE-IP-ADDR>`

and follow CLI-prompts.

The workflow is the following:  
1. Waiting for connection with smartphone and launching ROS `*.launch` script
2. Obtaining MCU-smartphone clock synchronization by Twist-n-Sync approach
3. Start recording. There is possibility to recording 
