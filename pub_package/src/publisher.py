import rospy
from std_msgs.msg import Int32

class NodeA:

    def __init__(self):
        rospy.init_node('nodeA')
            
        self.pub = rospy.Publisher('chithra', Int32, queue_size=10)
        self.rate = rospy.Rate(20)

        self.k, self.n = 1, 4

    def publish(self):
        while not rospy.is_shutdown():
            print("Pub to chithra", self.k)
            self.pub.publish(self.k)
            self.k += self.n
            self.rate.sleep()

if __name__ == '__main__':
    
    node = NodeA()
    node.publish()