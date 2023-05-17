#include "transform.h"

static inline bool is_inside(const cv::Mat &img, std::pair<int, int> p)
{
    return (p.first >= 0 && p.first < img.rows) && (p.second >= 0 && p.second < img.cols);
}

int rank_distance(uchar a, uchar b)
{
    return abs(a - b);
}

cv::Mat_<uchar> rank_transform(const cv::Mat_<uchar> &image)
{
    cv::Mat_<uchar> rank(image.rows, image.cols, (uchar)0);

    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {

            std::pair<int, int> start = {i - neighbourhood.first/2, j - neighbourhood.second/2};
            for (int k = 0; k < neighbourhood.first; ++k) {
                for (int l = 0; l < neighbourhood.second; ++l) {
                    if (is_inside(image, {start.first+k, start.second+l})
                        && image(start.first+k, start.second+l) < image(i, j)) {
                        rank(i, j)++;
                    }
                }
            }

        }
    }

    return rank;
}

int hamming_distance(int t1, int t2)
{
    int xor_res = t1 ^ t2;
    int ones = 0;

    while (xor_res > 0) {
        ones += xor_res & 1;
        xor_res >>= 1;
    }

    return ones;
}

cv::Mat_<int> census_transform(const cv::Mat_<uchar> &image)
{
    cv::Mat_<int> census(image.rows, image.cols, 0.0f);

    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {

            std::pair<int, int> start = {i - neighbourhood.first/2, j - neighbourhood.second/2};
            int pix_val = 0;
            for (int k = 0; k < neighbourhood.first; ++k) {
                for (int l = 0; l < neighbourhood.second; ++l) {
                    pix_val <<= 1;
                    if (!is_inside(image, {start.first+k, start.second+l})) {
                        pix_val |= 0;
                    } else {
                        pix_val |= image(start.first+k, start.second+l) < image(i, j);
                    }
                }
            }
            census(i, j) = pix_val;

        }
    }

    return census;
}