#!/usr/bin/env python
# FFT -> Filter -> IFFT and republish EMG signal from emg_stream topic to 
# fft_emg_stream topic

import rospy
import numpy as np
import scipy.fft
import collections
from shimmer_ros.msg import Fft, Emg

class EMGFFTFilter():
  def __init__(
    self, window_size_relative, queue_size, sampling_rate, hpf_cutoff,
    lpf_cutoff, band_stop, fft_threshold
    ) -> None:
    # Window size, sampling rate and filtering frequencies
    self.window_size_relative = window_size_relative
    self.sampling_rate = sampling_rate
    self.window_size = round(self.window_size_relative * self.sampling_rate)
    # Always round to nearest even number
    self.window_size += (self.window_size % 2)
    self.hpf_cutoff = hpf_cutoff
    self.lpf_cutoff = lpf_cutoff
    self.band_stop = band_stop
    self.fft_threshold = fft_threshold
    # Initialize publishers with queue size
    self.fft_pub = rospy.Publisher(
      'fft_emg_stream', Fft, queue_size=queue_size
      )
    # Initialize empty lists and msgs
    self.emg_ch1 = collections.deque()
    self.emg_ch2 = collections.deque()
    self.timestamp = collections.deque()
    self.fft_seq = 0
    self.frame_id = ''
    self.fft_msg = Fft()
  
  def process_emg_msgs(self, emg_data):
    # Acquire shimmer name only from first message
    if not self.frame_id:
      self.frame_id = emg_data.header.frame_id
    # Append EMG data to channel lists
    self.emg_ch1.append(emg_data.emg_ch1)
    self.emg_ch2.append(emg_data.emg_ch2)
    # Timestamp of data (keep only last one)
    self.timestamp.append(emg_data.header.stamp)
    if len(self.emg_ch1) >= self.window_size:
      self.fft_filter_publish()
      # Clean EMG and timestamp data lists
      self.emg_ch1.clear(), self.emg_ch2.clear(), self.timestamp.clear()
  
  def fft_filter_publish(self):
    # Increase counter
    self.fft_seq += 1
    # Compute fft for each channel
    emg_ch1_fft = scipy.fft.rfft(self.emg_ch1)
    emg_ch2_fft = scipy.fft.rfft(self.emg_ch2)
    # Calculate frequencies
    freqs = scipy.fft.rfftfreq(
      n = len(self.emg_ch1), d = 1. / self.sampling_rate
      )
    # High-pass filtering
    if self.hpf_cutoff:
      emg_ch1_fft[freqs <= self.hpf_cutoff] = 0
      emg_ch2_fft[freqs <= self.hpf_cutoff] = 0
    # Low-pass filtering
    if self.lpf_cutoff:
      emg_ch1_fft[freqs >= self.lpf_cutoff] = 0
      emg_ch2_fft[freqs >= self.lpf_cutoff] = 0
    # Filter out band stop frequencies
    for band in self.band_stop:
      emg_ch1_fft[np.logical_and(freqs >= band[0], freqs <= band[1])] = 0
      emg_ch2_fft[np.logical_and(freqs >= band[0], freqs <= band[1])] = 0
    # FFT thresholding with magnitude
    if self.fft_threshold:
      emg_ch1_fft_abs = np.abs(emg_ch1_fft)
      emg_ch1_fft[emg_ch1_fft_abs < self.fft_threshold*emg_ch1_fft_abs.max()] = 0
      emg_ch2_fft_abs = np.abs(emg_ch2_fft)
      emg_ch2_fft[emg_ch2_fft_abs < self.fft_threshold*emg_ch2_fft_abs.max()] = 0

    # Compute inverse FFT to reconstruct filtered signal
    emg_ch1_ifft = scipy.fft.irfft(emg_ch1_fft)
    emg_ch2_ifft = scipy.fft.irfft(emg_ch2_fft)
    # rospy.logdebug(f'IFFT len: {emg_ch1_ifft.shape}')
    # rospy.logdebug(f'Windows size: {self.window_size}')
    
    # Populate ROS msg with FFT and iFFT data and publish
    if not self.fft_msg.header.frame_id:
      self.fft_msg.header.frame_id = self.frame_id
    # Frequencies and original timestamp
    self.fft_msg.fft_freqs = freqs.tolist()
    self.fft_msg.orig_timestamp = self.timestamp
    self.fft_msg.header.stamp = self.timestamp.pop()
    # self.fft_msg.header.seq = self.fft_seq
    # Channel 1
    self.fft_msg.amp_emg_ch1 = np.abs(emg_ch1_fft).tolist()
    self.fft_msg.ifft_emg_ch1 = emg_ch1_ifft.tolist()
    # Channel 2
    self.fft_msg.amp_emg_ch2 = np.abs(emg_ch2_fft).tolist()
    self.fft_msg.ifft_emg_ch2 = emg_ch2_ifft.tolist()
    
    self.fft_pub.publish(self.fft_msg)

def filter_republish_emg() -> None:
  """
  ROS parameters:
    window_size_relative (float): window size for filtering
    repub_queue_size (int): queue size for republished topic with filtered EMG
  """
  #TODO: parameters documentation
  # Initialize node and Publisher
  rospy.init_node(
    'emg_fft_filter',
    anonymous=False,
    log_level=rospy.DEBUG,
    )
  try:
    # Get filter parameters
    window_size_relative = rospy.get_param('window_size_relative', 1)
    rospy.loginfo(f'Window size: {window_size_relative}')
    queue_size = rospy.get_param('repub_queue_size', 10)
    rospy.loginfo(f'Queue size: {queue_size}')
    sampling_rate = rospy.get_param('sampling_rate') # Hz
    rospy.loginfo(f'Sampling rate: {sampling_rate}')
    hpf_cutoff = rospy.get_param('hpf_cutoff', None)
    rospy.loginfo(f'High pass cutoff frequency: {hpf_cutoff}')
    lpf_cutoff = rospy.get_param('lpf_cutoff', None)
    rospy.loginfo(f'Low pass cutoff frequency: {lpf_cutoff}')
    band_stop = rospy.get_param('band_stop', list())
    rospy.loginfo(f'Band stop frequencies: {band_stop}')
    fft_threshold = rospy.get_param('fft_threshold', 0)
    rospy.loginfo(f'FFT relative threshold: {fft_threshold}')
    emg_fft = EMGFFTFilter(
      window_size_relative=window_size_relative, 
      queue_size=queue_size,
      sampling_rate=sampling_rate,
      hpf_cutoff=hpf_cutoff,
      lpf_cutoff=lpf_cutoff,
      band_stop=band_stop,
      fft_threshold=fft_threshold
      )
    subscriber = rospy.Subscriber(
      name='emg_stream', 
      data_class=Emg, 
      callback=emg_fft.process_emg_msgs)
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.logwarn('User interrupted execution!')
  except rospy.ROSException:
    print("Could not get parameter names!")
