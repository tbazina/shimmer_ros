# shimmer_ros
ROS interface to Shimmer3 EMG sensors. Data can be streamed from several Shimmer
sensors simultaneously. Synchronisation is performed using `rospy.Time.now()` and internal Shimmer 32 kHz timer.

## Setup
Find out MAC adress of Shimmer device:
`hcitool scan`

Run setup/bind_shimmer_to_rfcomm.sh with root privilegies and providing all
Shimmer MAC adresses as positional arguments.

Add your USER to dialout group:
`sudo usermod -a -G dialout USER`

## Start streaming EMG
Edit the shimmer configuration in `config/shimmer_config.yaml`.

Run the launch file with `num` number of simmers, e.g.:
`roslaunch shimmer_ros publish_emg_data.launch num:=2`

Visalize data using PlotJuggler with launch file (configuration in `config/plotjuggler_config.xml`):
`roslaunch shimmer_ros plotjuggler_shimmer.launch`
