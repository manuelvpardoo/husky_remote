from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from cv_bridge import CvBridge
import cv2
import csv

bag_dir = "/home/huanyu/Documents/rosbag/dimrem/0306_global"
output_path = "/home/huanyu/rosbag_out"
save_interval = 5 * 1_000_000_000  # nanoseconds

bridge = CvBridge()

csv_filename = "/home/huanyu/rosbag_out/path_data.csv"
csv_file = open(csv_filename, mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "X", "Y", "Z", "X", "Y", "Z", "W"])

# create reader instance and open for reading
with Reader(bag_dir) as reader:
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)
    # iterate over messages
    last_timestamp = 0
    for connection, timestamp, rawdata in reader.messages():
        if timestamp - last_timestamp > save_interval:
            print(f"Saving at {timestamp}")
            last_timestamp = timestamp
            # if connection.topic == "/camera_360/image_compressed":
            #     msg = deserialize_cdr(rawdata, connection.msgtype)
            #     try:
            #         cv_image = bridge.compressed_imgmsg_to_cv2(msg)
            #     except Exception as e:
            #         print(e)
            #     cv2.imwrite(f"/home/huanyu/rosbag_out/{str(timestamp)[:9]}.png", cv_image)

            if connection.topic == "/path":
                msg = deserialize_cdr(rawdata, connection.msgtype)
                
                for pose in msg.poses:
                    timestamp_sec = pose.header.stamp.sec
                    timestamp_nsec = pose.header.stamp.nanosec
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = pose.pose.position.z
                    qx = pose.pose.orientation.x
                    qy = pose.pose.orientation.y
                    qz = pose.pose.orientation.z
                    qw = pose.pose.orientation.w

                    # Write the data to the CSV file
                    csv_writer.writerow(
                        [f"{str(timestamp)[:9]}", x, y, z, qx, qy, qz, qw]
                    )


csv_file.close()
