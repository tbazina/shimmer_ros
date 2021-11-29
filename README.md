# shimmer_ros
ROS interface to Shimmer3 EMG sensors. Data can be streamed from several Shimmer
sensors simultaneously.

## Setup
Find out MAC adress of Shimmer device:
```hcitool scan```

Run setup/bind_shimmer_to_rfcomm.sh with root privilegies and providing all
Shimmer MAC adresses as positional arguments.

Add your USER to dialout group:
```sudo usermod -a -G dialout USER```