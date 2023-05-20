#include <iostream>

#include "stereo.h"
#include "transform.h"

int main()
{
    stereo::stereo_s<uchar> stereo_image("Images/Tsukuba/left.png", "Images/Tsukuba/right.png");
    stereo_image.display({"original - left", "original - right"}, {10, 10});

//    stereo::stereo_s<uchar> transformed_image(stereo_image, rank_transform);
//    transformed_image.display({"transformed - left", "transformed - right"}, {10, stereo_image.left.rows + 70});

    stereo::stereo_s<int> transformed_image(stereo_image, census_transform);
    stereo::correlation_s<int, int> corr(transformed_image, hamming_distance);
    stereo::disparity disparity(corr);

    cv::imshow("result", disparity.disparity_matrix);
    cv::waitKey();

    return 0;
}
