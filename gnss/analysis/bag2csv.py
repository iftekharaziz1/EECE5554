import bagpy
from bagpy import bagreader

def bag_to_csv(bag_file_path):
   
    bag = bagreader(bag_file_path)
    
    # Get the list of topics in the bag file
    topics = bag.topic_table
    print("Topics available in the bag file:")
    print(topics)
    
    for topic in bag.topics:
        csv_file = bag.message_by_topic(topic)
        print(f"Messages from {topic} have been saved to: {csv_file}")

if __name__ == "__main__":
    bag_file_path = '/home/iftekhar/EECE5554/lab2/data/occluded_move.bag'
    
    bag_to_csv(bag_file_path)
