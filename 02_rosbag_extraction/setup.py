from setuptools import setup, find_packages

setup(
    name='rosbag_extractor',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'imageio-ffmpeg',
        'numpy',
        'opencv-python',
        'rosbags',
        'tqdm',
    ],
    entry_points={
        'console_scripts': [
            'bag_info=rosbag_extractor.bag_info:main',
            'image_extractor=rosbag_extractor.extract_images:main',
            'imu_csv_extractor=rosbag_extractor.extract_imu_csv:main',
            'location_csv_extractor=rosbag_extractor.extract_location_csv:main',
            'mag_csv_extractor=rosbag_extractor.extract_mag_csv:main',
            'message_extractor=rosbag_extractor.extract_messages:main',
            'point_cloud_extractor=rosbag_extractor.extract_pcds:main',
        ],
    },
)
