# Entire FFT diagram.
Header header          # timestamp of the measurement
                       # frame_id is the name of the EMG sensor
float32[] amp_emg_ch1  # FFT Amplitudes of EMG reading from channel 1 in mV (real)
float32[] amp_emg_ch2  # FFT Amplitudes of EMG reading from channel 2 in mV (real)
float32[] fft_freqs    # FFT sample frequencies (real)
float32[] ifft_emg_ch1 # Inverse FFT for channel 1 (signal reconstruction) 
float32[] ifft_emg_ch2 # Inverse FFT for channel 2 (signal reconstruction) 
time[] orig_timestamp  # Original timestamp for signal reconstruction (inverse FFT)