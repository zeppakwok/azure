#include <iostream>
#include <opencv2/opencv.hpp>
#include <k4a/k4a.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

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

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL; // 设定参数
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true; // 原图和深度图同步才可用

    device.start_cameras(&config);

    k4a::image rgbImage;
    k4a::image depthImage;
    cv::Mat color_frame;
    cv::Mat depth_frame;
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

    while (true)
    {
        if (device.get_capture(&capture, std::chrono::milliseconds(0)))
        {
            rgbImage = capture.get_color_image();
            depthImage = capture.get_depth_image();

            color_frame = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, rgbImage.get_buffer());
            depth_frame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());

            depth_frame.convertTo(depth_frame, CV_16U, 1);

            // 创建点云对象
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            cloud->width = depth_frame.cols;
            cloud->height = depth_frame.rows;
            cloud->is_dense = false;
            cloud->points.resize(cloud->width * cloud->height);

            for (int y = 0; y < depth_frame.rows; ++y)
            {
                for (int x = 0; x < depth_frame.cols; ++x)
                {
                    ushort depth = depth_frame.at<ushort>(y, x);
                    if (depth > 0)
                    {
                        pcl::PointXYZRGB &point = cloud->points[y * cloud->width + x];
                        point.z = depth / 1000.0;                                         // 深度单位转为米
                        point.x = (x - rgbImage.get_width_pixels() / 2) * point.z / 525;  // 估算内参
                        point.y = (y - rgbImage.get_height_pixels() / 2) * point.z / 525; // 估算内参
                        // 设置颜色
                        Vec4b color = color_frame.at<Vec4b>(y, x);
                        point.r = color[2]; // BGR to RGB
                        point.g = color[1];
                        point.b = color[0];
                    }
                }
            }

            // 显示点云
            viewer.showCloud(cloud);
            cv::imshow("color", color_frame);
            cv::imshow("depth", depth_frame);

            capture.reset();
            if (waitKey(1) == 27)
                break;
        }
    }
    cv::destroyAllWindows();
    device.close();
    return 0;
}