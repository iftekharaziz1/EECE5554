import bagpy
from bagpy import bagreader

bag_file = '/home/iftekhar/Downloads/RSN/Lab5/lab5_cal.bag'

bag = bagreader(bag_file)

topics = bag.topic_table

print(topics)

for topic in bag.topics:
    data = bag.message_by_topic(topic)
    print(f"Data for {topic} is saved in: {data}")
