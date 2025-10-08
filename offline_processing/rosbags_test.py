from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)


# create reader instance and open for reading
with Reader('/home/arco3/Desktop/rosbags/rosbag2_2024_06_27-16_23_32') as reader:
    for connection, timestamp, rawdata in reader.messages():
         msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
         print(msg.header.frame_id)
