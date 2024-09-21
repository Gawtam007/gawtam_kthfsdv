import rospy
from std_msgs.msg import Int32, Float32

class NodeB:
    
    def __init__(self):
        rospy.init_node('nodeB')
        self.sub = rospy.Subscriber('chithra', Int32, self.callback)
        
        self.pub = rospy.Publisher('/kthfs/result', Float32, queue_size=10)
        self.rate = rospy.Rate(20)

        self.a = 0.0

    def callback(self, msg):
        print("Sub from chithra : ", msg.data)
        q = 0.15
        self.a = msg.data/q

    def publish(self):
        while not rospy.is_shutdown():
            print("Pub to result : ", self.a)
            self.pub.publish(self.a)
            self.rate.sleep()
   
if __name__ == '__main__':
    
    node = NodeB()
    node.publish()
    

