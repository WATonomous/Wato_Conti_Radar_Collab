# Radar Publisher

ROS Publisher node to listen in and process packets.

Simplified Packet Sniffer program to read from Radar devices. Implemented using the libpcap packet sniffer. Code format follow http://wiki.ros.org/CppStyleGuide

## Built by ROS

### Dependencies
(DEPRECATED) Run the following in the terminal to install the packet capture library we use
1. `sudo apt-get install libpcap0.8-dev`
### Setup and Compile
First, create a new directory to contain your catkin workspace. Place `radar_driver` in a folder called *src*. Your directory structure should look something like this:
`/home/catkin_ws/src/radar_driver/`

Next, run `catkin_make` in your workspace root to initialize your workspace. Once it has initialized with no errors, source the setup.bash file into your environment
```
/home/catkin_ws source devel/setup.bash
```
Note if you are using a different shell (.sh or .zsh), there will be a corresponding setup file. If you run into message dependency errors, clone the ros_msgs repo in the same directory and build again

## Software Architecture

The radar driver system will take in raw UDP packets over Ethernet from the radar, and output clean & clear
data for perception in the form of ROS messages. This is currently accomplished using 3 individual nodes & topics.
Below are the nodes, topics and manual launch instructions. If launching the nodes manually, there would be no namespace.
#### When running manually, make sure to run roscore!

### Nodes & Topics:
#### 1. radar_publisher
This will take in the raw radar data & convert to ROS messages. This does no processing, filtering or grouping. It is simply meant to convert the data types to something ROS can understand & manipulate. radar_NS is the provided argument to a launch file in order to allow multiple radars.

    Publishes To: `radar_NS/unfiltered_radar_packet`
##### Manual Instructions:
Run by specifying an interface and a Berkeley packet filter expression. Assuming
the radars are on Ethernet and interface enp3s0, and port 31122 the arguments would be:
```
rosrun radar_driver radar_publisher -e enp3s0 -p 31122 -c 1
```
The -p for the input port, -e for the Ethernet interface. -f is
an optional string representing an additional filter string options. -c is to indicate live capture (DEPRECATED PCAP).

#### 2. radar_processor
This will take in the radar ROS messages & covert the raw data into clean data for perception.
This first step is to group the data by timestamp to get a proper sense of the order of the data. Then we run some basic thresholding to eliminate the obviously unnecessary packets. The next step is to perform whatever processing/filtering is required to clean the data. Then output

    Subscribes To: `unfiltered_radar_packet`
    Publishes To: `radar_NS/filtered_radar_packet`
##### Manual Instructions:
This method only runs the radar_processor. The publisher or rosbag must be running first. Generally this method is used when testing different filtering algorithms, and so only the `radar_processor` node is run.
```
rosrun radar_driver radar_processor -t /radar/unfiltered_radar_packet
```
The -t can be included to specify the topic it is filtering on, the default is unfiltered_radar_packet. Most of our rosbags have the namespace radar, radar_left or radar_right. Therefore you must include this argument for the particular rosbag. They are named according to their namespace.
The -s can be included to change the scanmode. It always defaults to 0 (near and far), but setting it to 1 means near scan only and setting it to 2 means far scan only.

#### 3. multi_radar
This node takes in the filtered data from both radars and will set up communication between the multi-radar set-up. The purpose of this node
is to further filter out any more bad data and fuse the two point-clouds of the radars into one point-cloud.

    Subscribes To: `/radar_NS/filtered_radar_packet`
    Publishes To: `/radar_NS/radar_pointcloud`
##### Manual Instructions:
Currently there are no manual instructions for running the multi_radar node. The node declarations can be found in the all_radar.launch file.

NOTE: In the radarDetection.cpp file, the detection.size() must be
greater than one. The size is zero if the radars do not detect anything
when they first start up. This can be seen when the FOV lines are not seen. When the radars detect something, FOV lines are seen and the
size of the detections array is at least one. However when the radars do not detect anything for a while, the size of the detections array is still one.

#### 4. radar_visualizer
This will take the clean ROS Radar messages and convert to the SensorMsgs/PointCloud2 ROS Message type. This is so that the rviz visualizer we have can properly visualize the data.

    Subscribes To: `filtered_radar_packet`
    Publishes To: `/radar_NS/radar_pointcloud`
##### Manual Instructions:
The topic name is needed. This lets the visualizer show whatever data we want, that being from the unfiltered or filtered topic. The topic must already be publishing.
```
rosrun radar_driver radar_visualizer -t filtered_radar_packet
```
The -t must be included so visualizer knows which topic needs to be visualized.
Run rviz. There is a predefined configuration file to set up the cloud. This is at:
catkin_ws/src/radar_driver/radarPointCloud.rviz
```
rosrun rviz rviz -d radarPointCloud.rviz
```  

## Run

### Launch single radar live
##### With Visualizer
```
roslaunch radar_driver single_radar.launch name:=radar port:=31122 iface:='enp0s25'
```
##### Without Visualizer
```
roslaunch radar_driver single_radar.launch ... filter="'more settings'"
```
##### Notes
1. Find iface from typing `ifconfig` into terminal (first Ethernet interface, starts with 'e').
2. The name arg sets the namespace of all the nodes, and the port & iface args set the port & interface. Name defaults to 'radar'. Note that when recording rosbags, the publisher would be publishing to /radar_NS/unfiltered_radar_packet where radar_NS is the name specified. Name the rosbag accordingly.
3. The [filter] argument is optional, represents any additional packet string settings. It can be used as follows: The filter string must be surrounded by double quotes ("") and single quotes ('') in
either order. This is just however ROS implemented their args I guess.
4. The scanMode can be included to change the scan mode. It always defaults to 0 (near and far), but setting it to 1 means near scan only and setting it to 2 means far scan only.

### Launch single radar offline from rosbag
##### Option 1: Use the launch script (Must use for namespace)
This method is usually used when working offline.
```
roslaunch radar_driver radarROSbag.launch name:=radar_left bagPath:=/home/usr/rosbags/radar_walking_across.bag bagTopic:=/radar/unfiltered_radar_packet
```
The bagTopic has to be used only if the rosbag topic namespace is different than the current namespace specified. In the example above, we want to use the radar_left namespace but the rosbag has the radar namespace, therefore I had to specify the namespace using bagTopic!
The scanMode can be included to change the scan mode. It always defaults to 0 (near and far), but setting it to 1 means near scan only and setting it to 2 means far scan only.
Replace "usr" with whatever username you set up Linux with. For example if you named your computer "wato" then the bag path should be: /home/wato/rosbags/MRTest.bag

##### Option 2: Manually
This method only runs the radar_processor. Rosbags are recorded from output of the `/radar_NS/unfiltered_radar_packet` topic from the `radar_publisher` node. Generally this method is used when testing different filtering algorithms, and so only the `radar_processor` node is run.
```
# Run ROSBAG wherever it is
rosbag play /home/rosbags/radar_right_walking_away_directly_in_front.bag -lq  # -lq means loop & quiet
...
# Run processor node, visualizer if needed.
rosrun radar_driver radar_processor -t /radar/unfiltered_radar_packet
```
Once again -t has been used to specify the namespace. This is because we are running the node manually hence there is no namespace. radar_processor would be looking for /unfiltered_radar_packet but the rosbag is publishing to /radar_right/unfiltered_radar_packet.

### Run Multiple Radars

###Changing ports:
To run multiple radars, the python script "EthCfgRx.py" is used to change a radar's individual port number. This is done to ensure that the individual radars can receive data independently. To begin, connect only one radar with an Ethernet cable directly to the computer. Before running the script, run a network application such as Wireshark to find out the IP address of the radar, the MAC address of the radar, and the Ethernet interface. The MAC address has the following format: xx:xx:xx:xx:xx:xx where xx is a two digit hex number. After finding all the above information, here is an example of the command, where tx indicates the new port number you want to configure the radar to:

sudo python EthCfgRx.py -n enp0s25 -ip 192.168.1.2 -m 02:08:02:03:00:00 -tx 31121

The port numbers must be within 31115 to 31122, where 31122 is the default port number for a radar.

### Launch multi-radar offline from rosbag
##### Option 1: Use the launch script (Must use for namespace)
This method is usually used when working offline.
```
roslaunch radar_driver multiradarROSbag.launch bagPath:=/home/usr/rosbags/MRTest.bag
```
The bagTopic has to be used only if the rosbag topic namespace is different than the current namespace specified. For this specific bag, the bag topics are the unfiltered_radar_packet topics. So there is no need to specify the bagTopic in this example as unfiltered_radar_packet is the default topic for both radars.
The scanMode can be included to change the scanmode. It always defaults to 0 (near and far), but setting it to 1 means near scan only and setting it to 2 means far scan only.
Replace "usr" with whatever user name you set up Linux with. For example if you named your computer "wato" then the bag path should be: /home/wato/rosbags/MRTest.bag

##### With Visualizer
##### Without Visualizer
```
roslaunch radar_driver all_radar.launch --screen
```
Optional `--screen` to see output

## Trouble-shooting

### Permission errors:
If you are getting permission errors, run the following first on the linux terminal:
```
sudo setcap 'cap_net_raw=pe' devel/lib/radar_driver/radar_publisher
```
This will enable the radar_publisher executable (node) to have sufficient permissions
to listen to packets.

### Utility Script
In order to change topics inside a rosbag use the rename_topic.py unside utilities folder.
`
python rename_topic.py --filename pathToBag --oldtopic /unfiltered_radar_packet --newtopic /radar/unfiltered_radar_packet
`
The --newfilename can be used as well, defaults to filtered_currentname.

### Fall 2019 - Members

Arjun Narayan

Steven Tuer

Mitchell Hoyt

Saeejith Nair

Jonathan Thomas

Vivek Lasi

### Winter 2020 - Members

Jonathan Thomas

Frank Yan

Brandon Goh

Gokul Unnikrishnan
