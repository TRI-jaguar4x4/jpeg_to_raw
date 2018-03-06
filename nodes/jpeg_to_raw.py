# jpeg to raw decoder
#    subscribe to /image_raw/compressed and /camera_info
#    convert to raw
#    publish /image
#

import sys

import PIL
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class jpeg_to_raw(Node):
    def __init__(self):
        super().__init__('jpeg_to_raw')

        self.camInfoSub = self.create_subscription(CameraInfo, "camera_info", self.camInfoCallback)
        self.compressedSub = self.create_subscription(CompressedImage, "image_raw/compressed", self.compressedCallback)
        self.camInfo = None
        self.rawPublisher_ = self.create_publisher(Image, 'image')

    def camInfoCallback(self, msg):
        self.camInfo = msg

    def compressedCallback(self, msg):
        if self.camInfo is None:
            return

        try:
            im = PILImage.frombuffer("RGB",
                                     (self.camInfo.width,self.camInfo.height),
                                     bytes(msg.data), "jpeg", "RGB", "")
        except Exception as e:
            print ("Exception loading PILImage.frombuffer: ", e)

        self.rawmsg = Image()
        self.rawmsg.header = msg.header
        self.rawmsg.width = self.camInfo.width
        self.rawmsg.height = self.camInfo.height
        self.rawmsg.encoding = "rgb8"
        self.rawmsg.data = im.tobytes()
        self.rawmsg.step = int(len(self.rawmsg.data)/self.rawmsg.height)
        self.rawPublisher_.publish(self.rawmsg)

def main():
    rclpy.init(args=sys.argv)
    jpeg2raw = jpeg_to_raw()
    rclpy.spin(jpeg2raw)
    jpeg2raw.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
