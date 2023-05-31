#ifndef STREREO_DEPTH_STEREO_H
#define STREREO_DEPTH_STEREO_H

#include <opencv2/opencv.hpp>

namespace stereo
{
    template <typename T>
    struct stereo_s
    {
        cv::Mat_<T> left;
        cv::Mat_<T> right;

        stereo_s(cv::Mat_<T> left, cv::Mat_<T> right) : left(std::move(left)), right(std::move(right)) {}
        stereo_s(const std::string& img_left, const std::string& img_right);
        stereo_s(const stereo_s<uchar> &images, cv::Mat_<T> (*transform)(const cv::Mat_<uchar> &));

        void display(const std::pair<std::string, std::string>& window_names, std::pair<int, int> window_pos, int displacement = 20) const;
    };

    template <typename T, typename K>
    struct correlation_s
    {
        K ***mat;
        struct {
            int rows, cols, depth;
        } size;

        explicit correlation_s(int rows, int cols, int depth);
        explicit correlation_s(const stereo_s<T>& images, K (*distance)(T, T));

        K &at(int i, int j, int k);
        int find_correlation(int i, int j, std::pair<int, int> kernel_size);
    };

    struct disparity
    {
        cv::Mat_<uchar> disparity_matrix;

        explicit disparity(correlation_s<int, int> corr, std::pair<int, int> kernel_size);
    };

    struct calib
    {
        float cam0[3][3]{};
        float cam1[3][3]{};
        float doffs{};
        float baseline{};
        int width{}, height{};
        int ndisp{};
        int isint{};
        int vmin{}, vmax{};
        int dyavg{}, dymax{};

        calib() = default;
        explicit calib(const std::string &calib_file_name);
    };

    cv::Mat_<float> stereo_depth_image(const disparity& disparity, calib calib);

}

#endif //STREREO_DEPTH_STEREO_H
