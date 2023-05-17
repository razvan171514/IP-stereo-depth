//
// Created by razvan on 5/6/23.
//

#ifndef STREREO_DEPTH_TRANSFORM_H
#define STREREO_DEPTH_TRANSFORM_H

#include <opencv2/opencv.hpp>

const std::pair<int, int> neighbourhood = {5, 5};

int rank_distance(uchar a, uchar b);
cv::Mat_<uchar> rank_transform(const cv::Mat_<uchar> &image);

int hamming_distance(int t1, int t2);
cv::Mat_<int> census_transform(const cv::Mat_<uchar> &image);

#endif //STREREO_DEPTH_TRANSFORM_H
