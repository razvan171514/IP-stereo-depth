#include <iostream>

#include "stereo.h"
#include "transform.h"

cv::Mat_<uchar> compute_disparity1(stereo::correlation_s<int, int> &correlation)
{
    cv::Mat_<uchar> disparity(correlation.size.rows, correlation.size.cols);

    for (int i = 0; i < correlation.size.rows; ++i) {
        for (int j = 0; j < correlation.size.cols; ++j) {
            int k = correlation.find_correlation(i, j, {15, 15});
            std::cout << j-k << ' ';
            disparity(i, j) = j-k;
        }
        std::cout<<'\n';
    }

    return disparity;
}

int main()
{
    stereo::stereo_s<uchar> stereo_image("Images/Tsukuba/left.png", "Images/Tsukuba/right.png");
    stereo_image.display({"original - left", "original - right"}, {10, 10});

//    stereo::stereo_s<uchar> transformed_image(stereo_image, rank_transform);
//    transformed_image.display({"transformed - left", "transformed - right"}, {10, stereo_image.left.rows + 70});

    stereo::stereo_s<int> transformed_image(stereo_image, census_transform);
//    transformed_image.display({"transformed - left", "transformed - right"}, {10, stereo_image.left.rows + 70});

    stereo::correlation_s<int, int> corr(transformed_image, hamming_distance);

    cv::imshow("result", compute_disparity1(corr));
    cv::waitKey();

    return 0;
}
