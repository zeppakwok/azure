#include <stdio.h>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <k4a/k4a.hpp>
#include <cstdlib>

using namespace cv;
using namespace std;

int main()
{
    k4a::capture capture;
    const uint32_t device_count = k4a::device::get_installed_count(); // 查看设备个数
    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL; // 设定设备参数
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;

    device.start_cameras(&config);

    k4a::image rgbImage;
    k4a::image depthImage;

    cv::Mat color_frame;
    cv::Mat depth_frame;

    while (true)
    {
        if (device.get_capture(&capture, std::chrono::milliseconds(0)))
        {
            rgbImage = capture.get_color_image();
            depthImage = capture.get_depth_image();

            color_frame = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, rgbImage.get_buffer());
            depth_frame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());

            depth_frame.convertTo(depth_frame, CV_8U, 1);

            cv::imshow("color", color_frame);
            cv::imshow("depth", depth_frame);

            color_frame.release();
            depth_frame.release();

            capture.reset();
            if (waitKey(1) == 27)
                break;
        }
    }
    cv::destroyAllWindows();
    device.close();
    return 0;
}