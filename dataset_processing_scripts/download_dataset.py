import requests
import os
import time
import json
from tqdm import tqdm

def download_file(file_id, filename, directory, api_token, server_url, output_dir, pbar):
    url = f"{server_url}/api/access/datafile/{file_id}"
    headers = {"X-Dataverse-key": api_token} if api_token else {}
    
    filepath = os.path.join(output_dir, directory, filename)
    
    # Check if file already exists
    if os.path.isfile(filepath):
        existing_size = os.path.getsize(filepath)
        # Optionally, compare with expected filesize
        # response_head = requests.head(url, headers=headers, timeout=30)
        # expected_size = int(response_head.headers.get('content-length', 0))
        # if existing_size == expected_size:
        print(f"Skipping {filename}: already exists.")
        pbar.update(1)
        return
        # else:
        #     print(f"Re-downloading {filename}: file size mismatch.")
    
    try:
        response = requests.get(url, headers=headers, stream=True, timeout=30)
        if response.status_code == 200:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            total_size = int(response.headers.get('content-length', 0))
            
            with open(filepath, 'wb') as f, tqdm(
                desc=filename,
                total=total_size,
                unit='iB',
                unit_scale=True,
                unit_divisor=1024,
                leave=False
            ) as file_pbar:
                for chunk in response.iter_content(chunk_size=8192):
                    if chunk:
                        size = f.write(chunk)
                        file_pbar.update(size)
            print(f"Downloaded {filename}")
            pbar.update(1)
        else:
            print(f"Failed to download {filename}: {response.status_code}")
            pbar.update(1)
    except requests.exceptions.Timeout:
        print(f"Timeout occurred while downloading {filename}")
        pbar.update(1)
    except Exception as e:
        print(f"An error occurred while downloading {filename}: {e}")
        pbar.update(1)

def download_all_files(dataset_info, api_token, server_url, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Access files from the correct location in the dataset_info
    files = dataset_info['datasetVersion'].get('files', [])
    total_files = len(files)
    total_size = sum(file['dataFile'].get('filesize', 0) for file in files)

    print(f"Total files to download: {total_files}")
    print(f"Total size to download: {total_size / (1024**3):.2f} GB")

    with tqdm(total=total_files, desc="Overall Progress", unit="file") as pbar:
        for file in files:
            file_id = file['dataFile'].get('id')
            filename = file['dataFile'].get('filename')
            directory = file.get('directoryLabel', '')
            
            if file_id and filename:
                download_file(file_id, filename, directory, api_token, server_url, output_dir, pbar)
                
                # Add a small delay to avoid overwhelming the server
                time.sleep(1)
            else:
                print(f"Invalid file information: {file}")
                pbar.update(1)

# Set your API token, server URL, and output directory
DV_TOKEN = None
SERVER_URL = "https://dataverse.orc.gmu.edu"
OUTPUT_DIR = "/media/mocap/Data/dataverse_download/rosbags"

# Get the dataset information
persistent_id = "doi:10.13021/orc2020/SP577T"
export_url = f"{SERVER_URL}/api/datasets/export"
params = {
    "exporter": "dataverse_json",
    "persistentId": persistent_id
}
headers = {"X-Dataverse-key": DV_TOKEN} if DV_TOKEN else {}

print("Fetching dataset information...")
try:
    response = requests.get(export_url, params=params, headers=headers, timeout=30)
    response.raise_for_status()
    dataset_info = response.json()
    print("Dataset information retrieved successfully.")
    
    if 'datasetVersion' in dataset_info and 'files' in dataset_info['datasetVersion']:
        download_all_files(dataset_info, DV_TOKEN, SERVER_URL, OUTPUT_DIR)
    else:
        print("Error: 'files' key not found in dataset_info['datasetVersion']")
        print("Available keys in datasetVersion:", dataset_info['datasetVersion'].keys())
except requests.exceptions.Timeout:
    print("Timeout occurred while fetching dataset information.")
except requests.exceptions.HTTPError as http_err:
    print(f"HTTP error occurred: {http_err}")
    print(response.text)
except Exception as e:
    print(f"An unexpected error occurred: {e}")