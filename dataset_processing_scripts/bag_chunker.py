#!/usr/bin/env python3

import rosbag
import rospy
import os
import csv
import argparse
import logging
import hashlib
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime

def setup_logging(output_dir):
    log_file = os.path.join(output_dir, 'chunking.log')
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        handlers=[
                            logging.FileHandler(log_file),
                            logging.StreamHandler()
                        ])
    return logging.getLogger(__name__)

def get_midpoint(t1, t2):
    return t1 + (t2 - t1) / 2

def create_chunk_name(folder_name, chunk_timestamp, chunk_id):
    timestamp = datetime.fromtimestamp(chunk_timestamp)
    formatted_date = timestamp.strftime('%Y-%m-%d_%H-%M-%S')
    
    chunk_name = f"{folder_name}_{formatted_date}_chunk{chunk_id:04d}"
    
    return chunk_name

def compute_md5(file_path):
    hash_md5 = hashlib.md5()
    with open(file_path, 'rb') as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def validate_chunk(output_path, expected_count, logger):
    try:
        with rosbag.Bag(output_path, 'r') as bag:
            count = sum(1 for _ in bag.read_messages())
        if count >= expected_count:
            logger.info(f"Validation passed for {output_path} with {count} messages.")
            return True
        else:
            logger.warning(f"Validation failed for {output_path}. Expected at least {expected_count} messages, got {count}.")
            return False
    except Exception as e:
        logger.error(f"Validation error for {output_path}: {e}")
        return False

def process_bag(input_bag, output_dir, points_per_chunk):
    logger = logging.getLogger(__name__)
    folder_name = os.path.basename(os.path.dirname(input_bag))
    bag_name = os.path.splitext(os.path.basename(input_bag))[0]
    chunk_info = []

    logger.info(f"Processing bag: {input_bag}")

    with rosbag.Bag(input_bag, 'r') as bag:
        chunk_id = 0
        chunk_messages = []
        point_cloud_count = 0
        chunk_start_time = None
        last_point_cloud_time = None
        messages_iterator = iter(bag.read_messages())
        
        try:
            while True:
                topic, msg, t = next(messages_iterator)
                if topic == '/sensor_suite/ouster/points':
                    if chunk_start_time is None:
                        chunk_start_time = t
                    point_cloud_count += 1
                    last_point_cloud_time = t

                chunk_messages.append((topic, msg, t))

                if point_cloud_count == points_per_chunk:
                    # Determine chunk end time
                    chunk_end_time = last_point_cloud_time

                    chunk_timestamp = chunk_start_time.to_sec()
                    # chunk_name = f"{folder_name}_{datetime.fromtimestamp(chunk_timestamp).strftime('%Y%m%d%H%M%S')}_c{chunk_id:04d}"
                    chunk_name = create_chunk_name(folder_name, chunk_timestamp, chunk_id)
                    output_path = os.path.join(output_dir, f"{chunk_name}.bag")

                    with rosbag.Bag(output_path, 'w') as outbag:
                        for c_topic, c_msg, c_t in chunk_messages:
                            if c_t <= chunk_end_time:
                                outbag.write(c_topic, c_msg, c_t)

                    # Validate the written chunk
                    if validate_chunk(output_path, points_per_chunk, logger):
                        md5_hash = compute_md5(output_path)
                    else:
                        md5_hash = 'Validation Failed'

                    chunk_info.append({
                        'original_bag': input_bag,
                        'chunk_name': chunk_name,
                        'start_time': datetime.fromtimestamp(chunk_start_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S'),
                        'end_time': datetime.fromtimestamp(chunk_end_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S'),
                        'md5_hash': md5_hash
                    })

                    logger.info(f"Created chunk: {chunk_name}")

                    chunk_id += 1
                    chunk_messages = []
                    point_cloud_count = 0
                    chunk_start_time = None
        except StopIteration:
            pass

        # Handle remaining messages
        if chunk_messages:
            chunk_timestamp = chunk_start_time.to_sec() if chunk_start_time else datetime.now().timestamp()
            chunk_name = create_chunk_name(folder_name, chunk_timestamp, chunk_id)
            output_path = os.path.join(output_dir, f"{chunk_name}.bag")

            with rosbag.Bag(output_path, 'w') as outbag:
                for topic, msg, t in chunk_messages:
                    outbag.write(topic, msg, t)

            # Validate the written chunk
            if validate_chunk(output_path, 1, logger):
                md5_hash = compute_md5(output_path)
            else:
                md5_hash = 'Validation Failed'

            chunk_info.append({
                'original_bag': input_bag,
                'chunk_name': chunk_name,
                'start_time': datetime.fromtimestamp(chunk_timestamp).strftime('%Y-%m-%d %H:%M:%S'),
                'end_time': datetime.fromtimestamp(last_point_cloud_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S') if last_point_cloud_time else 'N/A',
                'md5_hash': md5_hash
            })

            logger.info(f"Created final chunk: {chunk_name}")

    return chunk_info

def main():
    parser = argparse.ArgumentParser(description='Split rosbag files into time-synchronized chunks.')
    parser.add_argument('input_dir', help='Directory containing the original rosbag files.')
    parser.add_argument('output_dir', help='Directory to store the chunked bag files.')
    parser.add_argument('--points_per_chunk', type=int, default=900, help='Number of point clouds per chunk')
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    logger = setup_logging(args.output_dir)

    input_bags = [os.path.join(args.input_dir, f) for f in os.listdir(args.input_dir) if f.endswith('.bag')]
    all_chunk_info = []

    logger.info(f"Starting to process {len(input_bags)} bag files")

    with ProcessPoolExecutor(max_workers=3) as executor:
        futures = [executor.submit(process_bag, bag, args.output_dir, args.points_per_chunk) for bag in input_bags]
        for future in futures:
            all_chunk_info.extend(future.result())

    # Write chunk information to CSV
    csv_path = os.path.join(args.output_dir, 'chunk_info.csv')
    with open(csv_path, 'w', newline='') as csvfile:
        fieldnames = ['original_bag', 'chunk_name', 'start_time', 'end_time', 'md5_hash']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for chunk in all_chunk_info:
            writer.writerow(chunk)

    logger.info(f"Chunk information saved to {csv_path}")
    logger.info("Chunking process completed")

if __name__ == '__main__':
    main()