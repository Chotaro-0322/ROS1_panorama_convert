#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>



class ImageProcessing{
public:
    ImageProcessing(){
        ros::NodeHandle node;
        sub = node.subscribe("/kodak/image_raw/compressed", 1, &ImageProcessing::publish_process, this);
        pub = node.advertise<sensor_msgs::Image>("panoramaImage",1);
    }

    void publish_process(const sensor_msgs::CompressedImage &data){
        cv_bridge::CvImagePtr cv_img;
        sensor_msgs::ImagePtr pano_img;
        cv::Mat half_img;
        cv_img = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
        half_img = resize_process(cv_img);
        pano_img = panorama_process(half_img);
        pub.publish(pano_img);
    }

    cv::Mat resize_process(const cv_bridge::CvImagePtr &img){
        // std::cout << "Resize process\n"; 
        cv::Mat half_image;
        cv::resize(img->image, half_image, cv::Size(1440, 1440));
        return half_image;
    }

    sensor_msgs::ImagePtr panorama_process(const cv::Mat &img){
        // std::cout << "panorama_process" << img.rows << "\n";
        int h_han = (int)(img.rows/2); // 元画像の中心までの高さ
        int width = h_han - 350; // 元画像の外側を切り取る
        int pano_width = 1500; // パノラマ画像の横幅
        double x, y, dx, dy;
        int x_int, y_int;
        int ch;
        sensor_msgs::ImagePtr out; // 結果を格納する.
        // std::cout << "sensor msg to cv::mat\n";
        // std::cout << "width, panowidth is " << width << "\n" <<  pano_width << "\n";
        cv::Mat dst = cv::Mat::zeros(width, pano_width, CV_8UC3); // 空の画像を用意

        for (double r = 0; r < width - 1; ++r){
            // std::cout << "r is " << r << "/n";
            for (double theta = 0; theta < pano_width - 1; ++theta){
                x = (r + h_han * 0.5) * cos((theta * 360 / pano_width)*(M_PI/180)) + h_han;
                y = (r + h_han * 0.5) * sin((theta * 360 / pano_width)*(M_PI/180)) + h_han;
                // std::cout << "theta is " <<  << "/n";
                dx = x - int(x);
                dy = y - int(y);

                x_int = static_cast<int>(x);
                y_int = static_cast<int>(y);

                if (x_int >= pano_width - 1 || y_int >= width - 1){ // はみ出た部分は治す.
                    x_int = x_int % (pano_width - 1);
                    y_int = y_int % (pano_width - 1);
                }
                for (ch = 0; ch < 3; ++ch){
                    dst.at<cv::Vec3b>(r, theta)[ch] = (1 - dx) * (1 - dy) * (img.at<cv::Vec3b>(y_int, x_int)[ch]) + dx * (1 - dy) * (img.at<cv::Vec3b>(y_int, x_int + 1)[ch])
                                                        + (1 - dx) * dy * (img.at<cv::Vec3b>(y_int + 1, x_int)[ch]) + dx * dy * (img.at<cv::Vec3b>(y_int + 1, x_int + 1)[ch]);
                }
            }
        }
        // std::cout << "end panorama_process\n";
        cv::flip(dst, dst, 1);
        out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
        return out;
    }
private:
    ros::Subscriber sub;
    ros::Publisher pub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "panorama_convert");
    ImageProcessing Imageprocessing;
    ros::spin();
}
