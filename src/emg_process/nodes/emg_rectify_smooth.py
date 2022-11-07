#!/usr/bin/env python
# Rectify and smooth acquired signal from inverse FFT

import rospy
import numpy as np
import itertools
from shimmer_ros.msg import Fft, Emg

def rolling_window(emg_arr, window_size):
  """Apply rolling window to 1D numpy array and return with shape 
  [length - window_size + 1, window_size]

  Args:
      emg_arr (np.array): emg signal to apply rolling window on
      window (int): size of the window

  Returns:
      2D np.array: array with window_size in each row
  """
  shape = emg_arr.shape[:-1] + (emg_arr.shape[-1] - window_size + 1, window_size)
  strides = emg_arr.strides + (emg_arr.strides[-1],)
  return np.lib.stride_tricks.as_strided(emg_arr, shape=shape, strides=strides)

class EMGRectifySmooth():
  def __init__(
    self, queue_size, window_size_sec, rectify, smooth, ema_decay, sampling_rate
    ) -> None:
    self.window_size = round(window_size_sec * sampling_rate)
    # Generate window for exponential smoothing
    self.window_ma = np.array([(1-ema_decay)**i for i in range(self.window_size)])
    self.window_ma = self.window_ma / self.window_ma.sum()
    self.rectify = rectify
    self.smooth = smooth
    self.sampling_rate = sampling_rate
    # Initialize publishers with queue size
    self.clean_signal_pub = rospy.Publisher(
      'clean_emg_stream', Emg, queue_size=queue_size
      )
    self.frame_id = ''
    self.clean_emg_msg = Emg()
    # Initialize lists for storing filtered data
    self.filt_emg_ch1 = []
    self.filt_emg_ch2 = []
    self.timestamp = []
  
  def process_fft_msgs(self, ifft_data):
    # Acquire shimmer name only from the first message
    if not self.frame_id:
      self.frame_id = ifft_data.header.frame_id
    # Data from iFFT
    self.filt_emg_ch1.append(ifft_data.ifft_emg_ch1)
    self.filt_emg_ch2.append(ifft_data.ifft_emg_ch2)
    # Timestamp of each data point
    self.timestamp.append(ifft_data.orig_timestamp)
    if len(self.timestamp) == 2:
      # Append window_size-1 samples from second last IFFT sample to last
      # to enable continuous windowing 
      self.rectify_smooth_publish(
        emg_ch1=np.fromiter(
          itertools.chain(
            self.filt_emg_ch1[0][-self.window_size+1:], self.filt_emg_ch1[1]
            ),
          dtype=np.float32
          ), 
        emg_ch2=np.fromiter(
          itertools.chain(
            self.filt_emg_ch2[0][-self.window_size+1:], self.filt_emg_ch2[1]
            ),
          dtype=np.float32
          ),
        # Only keep timestamp from last samples
        timestamp=self.timestamp[1]
      )
      # Delete first list element
      del self.filt_emg_ch1[0], self.filt_emg_ch2[0], self.timestamp[0]
  
  def rectify_smooth_publish(self, emg_ch1, emg_ch2, timestamp):
    # Rectify signal
    if self.rectify:
      emg_ch1 = np.abs(emg_ch1)
      emg_ch2 = np.abs(emg_ch2)
      # emg_ch1 = np.abs(emg_ch1)**(1/20)
      # emg_ch2 = np.abs(emg_ch2)**(1/20)
    
    # Create 2D array with windowed signal for smoothing
    if self.smooth:
      windowed_emg_ch1 = rolling_window(emg_ch1, self.window_size)
      windowed_emg_ch2 = rolling_window(emg_ch2, self.window_size)
    
    # Smooth signal using ma or rms
    if self.smooth == 'rms':
      emg_ch1 = np.sqrt(np.mean(np.square(windowed_emg_ch1), axis=-1))
      emg_ch2 = np.sqrt(np.mean(np.square(windowed_emg_ch2), axis=-1))
    elif self.smooth == 'ema':
      emg_ch1 = np.average(
        windowed_emg_ch1, axis=-1, weights=self.window_ma
        )
      emg_ch2 = np.average(
        windowed_emg_ch2, axis=-1, weights=self.window_ma
        )

    # Populate ROS EMG msg with clean EMG data and publish
    if not self.clean_emg_msg.header.frame_id:
      self.clean_emg_msg.header.frame_id = self.frame_id
    for i in range(len(timestamp)):
      # rospy.loginfo(f'Timestamp len {len(self.timestamp)}')
      self.clean_emg_msg.header.stamp = timestamp[i]
      # rospy.loginfo(f'Data len: {self.filt_emg_ch1.shape}')
      # rospy.loginfo(f'Window len: {self.window_size}')
      if self.smooth:
        self.clean_emg_msg.emg_ch1 = emg_ch1[i]
        self.clean_emg_msg.emg_ch2 = emg_ch2[i]
      else:
        self.clean_emg_msg.emg_ch1 = emg_ch1[self.window_size-1 + i]
        self.clean_emg_msg.emg_ch2 = emg_ch2[self.window_size-1 + i]
      self.clean_signal_pub.publish(self.clean_emg_msg)

def rectify_smooth_republish_emg() -> None:
  """
  ROS parameters:
    window_size_relative (float): window size for filtering
    repub_queue_size (int): queue size for republished topic with filtered EMG
  """
  # Initialize node and Publisher
  rospy.init_node(
    'rectify_smooth',
    anonymous=False,
    log_level=rospy.DEBUG,
    )
  try:
    # Get filter parameters
    window_size_sec = rospy.get_param('window_size_smoothing', None)
    rospy.loginfo(f'Window size: {window_size_sec}')
    queue_size = rospy.get_param('clean_queue_size', 1000)
    rospy.loginfo(f'Queue size: {queue_size}')
    rectify = rospy.get_param('rectify', False)
    rospy.loginfo(f'Rectify signal: {rectify}')
    smooth = rospy.get_param('smoothing', False)
    rospy.loginfo(f'Smoothing: {smooth}')
    ema_decay = rospy.get_param('ema_decay', 0)
    rospy.loginfo(f'Exponential moving average decay: {ema_decay}')
    sampling_rate = rospy.get_param('sampling_rate') # Hz
    rospy.loginfo(f'Sampling rate: {sampling_rate}')
    emg_rect_smooth = EMGRectifySmooth(
      # window_size_relative=window_size_relative, 
      queue_size=queue_size,
      window_size_sec=window_size_sec,
      rectify=rectify,
      smooth=smooth,
      ema_decay=ema_decay,
      sampling_rate=sampling_rate,
      )
    subscriber = rospy.Subscriber(
      name='fft_emg_stream', 
      data_class=Fft, 
      callback=emg_rect_smooth.process_fft_msgs)
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.logwarn('User interrupted execution!')
  except rospy.ROSException:
    print("Could not get parameter names!")
