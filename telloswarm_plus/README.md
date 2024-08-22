## Multi-threaded Python API for Tello EDU


- `telloserver_EDU.py` provides an API for swarming / formation control over 2.45 GHz WLAN 
- `telloserver_video.py` [optional] enables video streaming from multiple drones
- `mocap.py` [optional] provides an API to get the drones' poses from a motion capture system (VICON) 
- `mocap.py` relies on `rospy` and [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) 

### Network Configuration

- use `python set-ap-mode.py -s [SSID] -p [PASSWORD]` to put each drones on a router. Then, set a static IP address to each drone in the DHCP settings of the router. This step is done only once. 

- use the static `IP` address of each drone, the drone's `name` in the VICON Tracker, and a `port` for video streaming in a dictionary as follows:

```python
DronesDict = {
    "Name": (IP, Port),
}
```

## How to use

see `test_api.py` as a sample. 