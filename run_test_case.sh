
publish_color_images(){
    roscd ros_images_publisher; ROOT=$PWD
    rosrun ros_images_publisher publish_images.py \
        --images_folder $ROOT/data/color/ \
        --topic_name test_data/color \
        --camera_info_file $ROOT/data/cam_params_realsense.json
}

publish_depth_images(){
    roscd ros_images_publisher; ROOT=$PWD
    rosrun ros_images_publisher publish_images.py \
        --images_folder $ROOT/data/depth/ \
        --topic_name test_data/depth \
        --camera_info_file "" \
        --format depth
}

set_camera_at_origin(){
    rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map head_camera 10
}

run_rviz(){
    roscd ros_images_publisher; ROOT=$PWD
    rosrun rviz rviz -d $ROOT/rviz_for_color_and_depth_images.rviz
}

# https://stackoverflow.com/questions/3004811/how-do-you-run-multiple-programs-in-parallel-from-a-bash-script
# Run above 4 functions in parallel:
(trap 'kill 0' SIGINT; publish_color_images & publish_depth_images & set_camera_at_origin & run_rviz)
