import cv2
import hashlib
import colorsys
import cv_bridge

import numpy as np

from py_trees.composites import Sequence, Selector
from ros_trees.leaves_ros import PublisherLeaf

from sensor_msgs.msg import Image, CompressedImage

class VisualiseObservation(PublisherLeaf):
  def __init__(self, name=None, topic_name='/observation/visualised', topic_class=Image, draw_bounding_box=True, draw_overlay=True, *args, **kwargs):
    super(VisualiseObservation, self).__init__(
      name=name if name else 'Visualise observation',
      topic_name=topic_name,
      topic_class=topic_class,
      load_fn=self.load_fn,
      save=False,
      *args,
      **kwargs)

    self.draw_bounding_box = draw_bounding_box
    self.draw_overlay = draw_overlay

    self.bridge = cv_bridge.CvBridge()

  def load_fn(self):
    observation = self._default_load_fn(auto_generate=False)
    
    image = self.bridge.imgmsg_to_cv2(observation.rgb_image, desired_encoding='passthrough')
    image = image.copy()
    
    for detection in observation.detections:
      color = self.colorize(detection.class_label)
      
      if self.draw_bounding_box:
        image = cv2.rectangle(
          image,
          (detection.x_left, detection.y_top),
          (detection.x_left + detection.width, detection.y_top + detection.height),
          color,
          2
        )
        
        text_size, _ = cv2.getTextSize(detection.class_label, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)

        image = cv2.rectangle(
          image,
          (detection.x_left, detection.y_top-5-text_size[1]),
          (detection.x_left + text_size[0], detection.y_top-2),
          color,
          -1
        )

        cv2.putText(image, detection.class_label, (detection.x_left, detection.y_top-4), cv2.FONT_HERSHEY_SIMPLEX, 0.3, 0, 1)

      if self.draw_overlay:
        roi = image[detection.y_top:detection.y_top+detection.height,detection.x_left:detection.x_left+detection.width,:]
        
        overlay = np.zeros((detection.height, detection.width, 3), np.uint8)
        overlay[:,:] = color

        overlay = cv2.bitwise_and(overlay, overlay, mask=self.bridge.imgmsg_to_cv2(detection.cropped_mask, desired_encoding='passthrough'))

        cv2.addWeighted(overlay, 0.3, roi, 1.0 - 0.3, 0.0, roi)
    
    return self.create_image(image)

  def create_image(self, image):
    return self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
    
  def colorize(self, word):
    # https://medium.com/apollo-data-solutions-blog/mapping-words-to-colors-cfa23a65d8c4

    m = hashlib.md5()
    m.update(word.encode())
    byte_digest = m.digest()

    first_num = ord(byte_digest[0])   # get int value of first character 0-255
    second_num = ord(byte_digest[1])   # get int value of second character 0-255
    mapped_num = first_num / 255.0 * 360  # map from 255 range to 360 range for degrees
  
    hue = (mapped_num % 360) / 360.0  # hue as percentage
    variation = second_num / 255.0 / 2 - 0.25  # add some limited randomness to saturation and brightness
    saturation = min(0.8 + variation, 1.0)  # will vary from 0.55 to 1.0
    brightness = min(0.7 + variation, 1.0)  # will vary from 0.45 to 0.95

    return [int(color * 255) for color in colorsys.hsv_to_rgb(hue, saturation, brightness)]

class VisualiseObservationCompressed(VisualiseObservation):
  def __init__(self, name='Visualise observation', topic_name='/observation/visualised/compressed', *args, **kwargs):
    super(VisualiseObservationCompressed, self).__init__(
      name=name,
      topic_name=topic_name,
      topic_class=CompressedImage,
      *args,
      **kwargs
    )
    self.bridge = cv_bridge.CvBridge()

  def create_image(self, image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return self.bridge.cv2_to_compressed_imgmsg(image)
