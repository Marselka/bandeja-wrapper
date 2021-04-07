## samsung-avatar-dataset-sync


Uses ROS, **OpenCamera Sensors** remote control API and **Twist-n-Sync** synchronization approach to record synchronized data from heterogenious hardware-software system.

### Usage
Connect all the sensors needed and start recording by running  
`master.py <SMARTPHONE-IP-ADDR>`  
and follow CLI-prompts.

`<SMARTPHONE-IP-ADDR>` is the smartphone IP-address shown as a prompt in **OpenCamera Sensors** mobile app. Computer and smartphone must be connected to the same WiFi-network.

The workflow is the following:  
1. Waiting for connection with smartphone and launching ROS `*.launch` script
2. Obtaining MCU-smartphone clock synchronization by Twist-n-Sync approach
3. Start recording. There is possibility to recording have timestamps when operator tap Enter. This is useful for creating susequences while extracting data by `bag-extractor`

### Obtained data
Recorded `*.bag` will appear in the CLI current directory.
Recorded video form smartphone along with metadata will be stored on smartphone. It is needed to manually download them to storage.

### Data extraction
Use `bag-extractor`

### Installation
`pip install -r requirements.txt`

### Troubleshooting

- If after stopping script by Ctrl+C there message `MASTER MESSAGE: Exiting` appears, but CLI doesn't give access to new commands, wait -- probably Azure Kinect sensor consumes time for stopping its processes.  
- If after stopping script by Ctrl+C there is no message `MASTER MESSAGE: Exiting`, but you see last word `done`, just tap Enter to get access to CLI.
