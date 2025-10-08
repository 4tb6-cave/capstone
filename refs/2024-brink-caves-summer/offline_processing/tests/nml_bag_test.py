
# Imports.
from os import sep
#import tempfile
from pprint import pprint
#import rosbag2_py
#import rclpy.clock
#from example_interfaces import msg
#from rclpy.serialization import serialize_message
from nml_bag import Reader


def main():
    """ Run an example to illustrate use of the bag Reader class.
    """
    
    # Initialize a bag reader.
    reader = Reader('/home/arco3/Desktop/rosbags/rosbag2_2024_06_27-16_23_32', 
                    storage_id='sqlite3', 
                    serialization_format='cdr')
    
    # Print the topics.
    print(f'The bag contains the following topics:')
    pprint(reader.topics)
    
    # Print the mapping between topics and message types.
    print(f'The message types associated with each topic are as follows:')
    pprint(reader.type_map)
    
    # Print the messages stored in the bag.
    #print(f'All message data records:')
    ##pprint(reader.records)
    
    # Return
    return


main()