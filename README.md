## samsung-avatar-dataset-sync


Uses ROS, [**OpenCamera Sensors**](https://github.com/MobileRoboticsSkoltech/OpenCamera-Sensors) remote control API and [**Twist-n-Sync**](https://github.com/MobileRoboticsSkoltech/twistnsync-python)<sup>1</sup> synchronization approach to record synchronized data from heterogenious hardware-software system.

### Installation
`pip install -r requirements.txt`

### Usage
Connect all the sensors needed launch OpenCamera Sensors mobile app and launch  
`master.py <SMARTPHONE-IP-ADDR>`  
then follow CLI-prompts to start recording.

`<SMARTPHONE-IP-ADDR>` is the smartphone IP-address shown as a prompt in OpenCamera Sensors. Computer and smartphone must be connected to the same WiFi-network.

The workflow is the following:  
1. Waiting for (i) connection with smartphone and (ii) launching ROS `*.launch` script
2. Obtaining MCU-smartphone clock synchronization by Twist-n-Sync approach
3. Start recording. An operator may tap Enter every time when wish to create timestamps for indiactiing some events. This is useful for creating sub-sequences while extracting data by `bag-extractor`.

### Obtained data
Recorded `*.bag` will appear in the CLI current directory.
Recorded smartphone video along with metadata should be manually downloaded to the computer.

### Data extraction
Use `bag-extractor` and its instructions.

### Troubleshooting

- If after stopping script by Ctrl+C there message `MASTER MESSAGE: Exiting` appears, but CLI doesn't give access to new commands, wait -- probably Azure Kinect sensor consumes time for stopping its processes.  
- If after stopping script by Ctrl+C there is no message `MASTER MESSAGE: Exiting`, but you see last word `done`, just tap Enter to get access to CLI.
- To avoid problems with Azure, while working by SSH, run `export DISPLAY=:0` before starting the script every time connecting via SSH. Automotic user login must be set to ON.

<sup>1</sup>
```
@article{faizullin2021twist,
  title={Twist-n-Sync: Software Clock Synchronization with Microseconds Accuracy Using MEMS-Gyroscopes},
  author={Faizullin, Marsel and Kornilova, Anastasiia and Akhmetyanov, Azat and Ferrer, Gonzalo},
  journal={Sensors},
  volume={21},
  number={1},
  pages={68},
  year={2021},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```
