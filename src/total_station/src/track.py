from total_station.Test import Test
import time
import rospy
from total_station.track_fd import Track
from total_station.msg import totalStation

# float64 x
# float64 y
# float64 z
# float64 time
# int32 status

class DroneTracker(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(30)
        self.pub_total_station = rospy.Publisher('total_station/dron_loc', totalStation, queue_size=2)
        self.sub_drone_loc = rospy.Subscriber("/drone/loc_for_total_station", totalStation, self.callback)

        self.osdk_loc = None
        self.total_station_drone_loc = None
        self.track = Track()

    def callback(self, msg):
        self.osdk_loc.x = msg.x
        self.osdk_loc.y = msg.y
        self.osdk_loc.z = msg.z
        self.osdk_loc.time = msg.time
        self.osdk_loc.status = msg.status
    
    def run(self):
        # while not rospy.is_shutdown():
        self.track.open_port("/dev/rfcomm0", 9600)
        self.track.get_station_coord()

        while True: #while program not interrupted by the user
            # t_start = time.time()

            line2 = self.track.get_measure()
            line2 = line2 + "," + str("%f" % time.time()) + '\n'
            print(line2)
            if line2[0] == '0':
                file_save.write(line2)
            if line2[0] == '1':
                file_save.write(line2)
            # print(self.compute_azimuth_and_elevation())
            # print(self.station_coord)
        
            # t_end = time.time()
            # print(t_end-t_start)

            self.pub_total_station.publish(self.total_station_drone_loc)
            # self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("total_station_node", anonymous=True)


    file_name = str(time.time()) + '.txt'
    file_save = open(file_name, 'w', buffering=1)
    line = 'response_code,x,y,z,timestamp\n'
    file_save.write(line)

    
    tracker = DroneTracker()
    tracker.run()
