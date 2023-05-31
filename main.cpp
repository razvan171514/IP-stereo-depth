#include <iostream>

#include "stereo.h"
#include "transform.h"

using namespace cv;

static inline bool is_inside(const cv::Mat &img, std::pair<int, int> p)
{
    return (p.first >= 0 && p.first < img.rows) && (p.second >= 0 && p.second < img.cols);
}

Mat_<uchar> median_filter(const Mat_<uchar> &src, std::pair<int, int> kernel_size)
{
    Mat_<uchar> dst(src.rows, src.cols);

    for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {

            std::vector<int> vec;
            for (int di = 0; di < kernel_size.first; ++di) {
                for (int dj = 0; dj < kernel_size.second; ++dj) {
                    std::pair<int, int> curr = {i + di, j + dj};
                    if (is_inside(src, curr))
                        vec.push_back(src(curr.first, curr.second));
                }
            }

            sort(vec.begin(), vec.end());
            dst(i, j) = vec[vec.size()/2];
        }
    }

    return dst;
}

double compute_difference_quality(const Mat &img1, const Mat &img2)
{
    double sum = 0;
    for (int i = 0; i < img1.rows; ++i) {
        for (int j = 0; j < img2.cols; ++j) {
            int diff = img1.at<uchar>(i, j) - img2.at<uchar>(i, j);
//            sum += sqrt((img1.at<uchar>(i, j) * img1.at<uchar>(i, j)) + (img2.at<uchar>(i, j) * img2.at<uchar>(i, j)));
            sum += diff > 10;
        }
    }
    return sum / (img1.rows * img1.cols);
}

int main()
{
    Mat left = imread("Images/Adirondack/img0.jpg", IMREAD_GRAYSCALE);
    Mat right = imread("Images/Adirondack/img1.jpg", IMREAD_GRAYSCALE);

    Mat resize_left, resize_right;
    resize(left, resize_left, Size(), 0.5, 0.5);
    resize(right, resize_right, Size(), 0.5, 0.5);

    stereo::calib calibration("Images/Adirondack/calib.txt");

    stereo::stereo_s<uchar> stereo_image(resize_left, resize_right);
    stereo_image.display({"original - left", "original - right"}, {10, 10});

//    stereo::stereo_s<uchar> transformed_image(stereo_image, rank_transform);
//    transformed_image.display({"transformed - left", "transformed - right"}, {10, stereo_image.left.rows + 70});

    stereo::stereo_s<int> transformed_image(stereo_image, census_transform);
    stereo::correlation_s<int, int> corr(transformed_image, hamming_distance);
    stereo::disparity disparity(corr, {1, 1});

    Mat_<uchar> resulting_image = median_filter(disparity.disparity_matrix, {5, 5});

    imwrite("Images/Adirondack/disparity.jpg", resulting_image);
    cv::imshow("result", resulting_image);
    cv::waitKey();

    Mat_<uchar> my_disparity = imread("Images/tsukuba-2001/disparity.jpg", IMREAD_GRAYSCALE);
    Mat_<uchar> actual_disparity = imread("Images/tsukuba-2001/truedisp.row3.col3_1.jpg", IMREAD_GRAYSCALE);

    std::cout << compute_difference_quality(my_disparity, actual_disparity);



    return 0;
}
