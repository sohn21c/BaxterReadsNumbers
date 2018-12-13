"""
Basic node that simply reads in calibrated camera info from a YAML file and
publishes on a camera_info topic. For more detailed descriptions of all
parameters, see the documentation for camera_info_manager_py

PUBLISHERS:
    ~ camera_info (sensor_msgs/CameraInfo)

SUBSCRIBERS:
    ~ None

SERVICES:
    ~ set_camera_info (sensor_msgs/SetCameraInfo) -- inherited from camera_info_manager_py

PARAMETERS:
    ~ freq (float) -- frequency that we want to publish topic at
    ~ url (string) -- path to read the camera calibration info (optional)
    ~ cname (string) -- camera name
    ~ namespace (string) -- namespace to prefix service with.
"""

import rospy
import camera_info_manager as cim

DEFAULT_FREQUENCY = 10 # Hz
DEFAULT_CNAME = 'camera'
DEFAULT_URL = ''
DEFAULT_NAMESPACE = ''

class CameraInfoNode( object ):
    def __init__(self):
        self.freq = rospy.get_param("~freq", DEFAULT_FREQUENCY)
        self.cname = rospy.get_param("~cname", DEFAULT_CNAME)
        self.url = rospy.get_param("~url", DEFAULT_URL)
        self.namespace = rospy.get_param("~namespace", DEFAULT_NAMESPACE)

        self.cam_manager = cim.CameraInfoManager(
            cname=self.cname,
            url=self.url,
            namespace=self.namespace)

        try:
            self.cam_manager.loadCameraInfo()
            rospy.loginfo("Camera name = %s", self.cam_manager.getCameraName())
        except IOError as e:
            rospy.logerr("Tried to load camera calibration data, but it is unreadable. Error = %s", e.message)
            rospy.signal_shutdown("Could not read calibration info")
        except cim.CameraInfoMissingError as e:
            rospy.logerr("Camera info missing. Error = %s", e.message)
            rospy.logerr("Node will continue to run, but you will need to call set_camera_info service to fill out camera data")

        try:
            if not self.cam_manager.isCalibrated():
                rospy.logwarn("No exceptions during startup, but camera calibration data is using default values!")
                rospy.loginfo("Calibration info: ")
                print(self.cam_manager.getCameraInfo())
            else:
                rospy.loginfo("No errors during the loading of the camera data -- should be publishing okay.")
        except cim.CameraInfoMissingError as e:
            rospy.logerr("Even after trying to load camera info, we still don't have calibration information available")
            rospy.signal_shutdown("Can't even provide default calibration data... shutting down")

        self.pub = rospy.Publisher("camera_info", cim.CameraInfo, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.freq), self.timercb)

        return

    def timercb(self, tevent):
        try:
            cinfo = self.cam_manager.getCameraInfo()
            self.pub.publish(cinfo)
        except cim.CameraInfoMissingError as e:
            rospy.logwarn_throttle(rospy.Duration(1.0), "No camera calibration info available! Error = %s", e.message)
        return


def main():
    rospy.init_node("camera_info_manager_node", log_level=rospy.INFO)

    try:
        cin = CameraInfoNode()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    return


if __name__ == '__main__':
    main()
