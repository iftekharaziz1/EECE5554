import bagpy
from bagpy import bagreader

def bag_to_csv(bag_file_path):
    # Initialize the bag reader with the specified bag file
    bag = bagreader(bag_file_path)
    
    # Get the list of topics in the bag file
    topics = bag.topic_table
    print("Topics available in the bag file:")
    print(topics)
    
    # Iterate over each topic and save its messages to a CSV file
    for topic in bag.topics:
        csv_file = bag.message_by_topic(topic)
        print(f"Messages from {topic} have been saved to: {csv_file}")

if __name__ == "__main__":
    # Replace this path with the path to your ROS bag file
    bag_file_path = '/home/iftekhar/EECE5554/gnss/Data/occluded_1.bag'
    
    # Convert the bag file to CSV files
    bag_to_csv(bag_file_path)
