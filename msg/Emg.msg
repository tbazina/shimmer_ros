# Single EMG signal reading in mV. This message is appropriate for measuring
# the surface EMG signal from human body. 
Header header           # timestamp of the measurement
                        # frame_id is the name of the EMG sensor
float32 emg_ch1     # Absolute EMG reading from channel 1 in mV
float32 emg_ch2     # Absolute EMG reading from channel 2 in mV