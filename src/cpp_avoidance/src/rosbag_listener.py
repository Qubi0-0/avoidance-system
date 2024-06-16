import rospy
import time

class MessageListener:
    def __init__(self, topic):
        self.topic = topic
        self.last_timestamp = None
        self.total_period = 0.0
        self.message_count = 0

    def callback(self, msg):
        current_timestamp = rospy.get_time()
        if self.last_timestamp is not None:
            self.total_period += current_timestamp - self.last_timestamp
            self.message_count += 1
        self.last_timestamp = current_timestamp

    def compute_average_period(self):
        if self.message_count == 0:
            return None
        return self.total_period / self.message_count

def main():
    rospy.init_node('message_listener', anonymous=True)
    topics = ["/octomap_binary"]
    listeners = {topic: MessageListener(topic) for topic in topics}

    for topic, listener in listeners.items():
        rospy.Subscriber(topic, rospy.AnyMsg, listener.callback)

    while not rospy.is_shutdown():
        time.sleep(1) 
        total_average_period = 0.0
        valid_topics_count = 0
        for topic, listener in listeners.items():
            average_period = listener.compute_average_period()
            if average_period is not None:
                rospy.loginfo("Average period for topic %s: %f", topic, average_period)
                total_average_period += average_period
                valid_topics_count += 1
        if valid_topics_count > 0:
            rospy.loginfo("Total average period: %f", total_average_period)

if __name__ == '__main__':
    main()