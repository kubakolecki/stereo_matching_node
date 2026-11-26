import os
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_bm',
            executable='stereo_bm',
            name='stereo_bm',
            parameters=[{'input_topic_name': '/orbslam3/georeferenced_stereo_image'},
                        {'do_save_parallax_maps': False},
                        {'do_save_depth_maps': False},
                        {'do_save_local_point_clouds': True},
                        {'do_save_global_point_clouds': False},
                        {'path_to_save_parallax_map': '/datadisk/data/agh_projects/20260601_depth_map_datasets/variant_depth_maps_only_test_01/stereo_bm/2026_06_01-11_54_05'},
                        {'path_to_save_depth_maps': '/datadisk/data/agh_projects/20260601_depth_map_datasets/variant_depth_maps_only_test_01/stereo_bm/2026_06_01-11_54_05'},
                        {'path_to_save_local_point_clouds': '/datadisk/data/agh_projects/20260601_depth_map_datasets/variant_depth_maps_only_test_01/stereo_bm/2026_06_01-11_54_05'},
                        {'path_to_save_global_point_clouds': '/datadisk/data/agh_projects/20260601_depth_map_datasets/variant_depth_maps_only_test_01/stereo_bm/2026_06_01-11_54_05'},
                        {'roi_row_min': 40},
                        {'roi_row_max': 910},
                        {'roi_col_min': 88},
                        {'roi_col_max': 1270}]
                        ,
            output='screen',

            emulate_tty=True
        )
    ])
