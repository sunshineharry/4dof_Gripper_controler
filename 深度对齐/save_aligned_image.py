# 显示相机的深度图和彩色图，效果和Intel配套软件相同
# 【注】 相机的第一张图像会出现色彩失真的情况，但后续图像正常
# 


import pyrealsense2 as rs
import numpy as np
import cv2
import datetime

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

# Start streaming
pipeline.start(config)
# 像素对齐使用 rs.align 模块
align = rs.align(rs.stream.color)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        ir_frame_left = frames.get_infrared_frame(1)
        ir_frame_right = frames.get_infrared_frame(2)
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        # np.asanyarray 会返回 ndarray 或者 ndarray的子类
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        ir_left_image = np.asanyarray(ir_frame_left.get_data())
        ir_right_image = np.asanyarray(ir_frame_right.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images1 = np.hstack((color_image, depth_colormap))
        images2 = np.hstack((ir_left_image, ir_right_image))
        image3 = cv2.addWeighted(color_image,0.7,depth_colormap,0.3,0)

        # 像素对齐
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        if not aligned_color_frame or not aligned_depth_frame:
            continue

        aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
        aligned_color_image = np.asanyarray(aligned_color_frame.get_data())

        aligned_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(aligned_depth_image, alpha=0.1), cv2.COLORMAP_JET)
        image4 = cv2.addWeighted(aligned_color_image,0.7,aligned_depth_colormap,0.3,0)


        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images1)
        # cv2.imshow("Display pic_irt", images2)
        cv2.imshow("Merge image",image3)
        cv2.imshow("Merge image_aligned",image4)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            ISOTIMEFORMAT = '%Y_%m_%d_%H_%M_%S'
            theTime = datetime.datetime.now().strftime(ISOTIMEFORMAT)
            cv2.imwrite(str(theTime)+'color_image_'+'.png',color_image)
            cv2.imwrite(str(theTime)+'depth_colormap_'+'.png',depth_colormap)
            cv2.imwrite(str(theTime)+'depth_'+'.png',depth_image)
            cv2.imwrite(str(theTime)+'merge'+'.png',image3)
            cv2.imwrite(str(theTime)+'aligned_depth_colormap'+'.png',aligned_depth_colormap)
            cv2.imwrite(str(theTime)+'aligned_color_image'+'.png',aligned_color_image)
            cv2.imwrite(str(theTime)+'aligned_depth_'+'.png',aligned_depth_image)
            cv2.imwrite(str(theTime)+'aligned_merge'+'.png',image4)
            break

finally:
    # Stop streaming
    pipeline.stop()