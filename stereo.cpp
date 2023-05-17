#include "stereo.h"

template stereo::stereo_s<uchar>::stereo_s(const std::string &img_left, const std::string &img_right);
template stereo::stereo_s<uchar>::stereo_s(
        const stereo::stereo_s<uchar> &images, cv::Mat_<uchar> (*transform)(const cv::Mat_<uchar> &)
);
template void stereo::stereo_s<uchar>::display(
        const std::pair<std::string, std::string>& window_names, std::pair<int, int> window_pos, int displacement
) const;

template stereo::correlation_s<uchar, int>::correlation_s(int rows, int cols, int depth);
template stereo::correlation_s<uchar, int>::correlation_s(const stereo::stereo_s<uchar> &images, int (*distance)(uchar, uchar));
template int stereo::correlation_s<uchar, int>::find_correlation(int i, int j, std::pair<int, int> kernel_size);


template stereo::stereo_s<int>::stereo_s(
        const stereo::stereo_s<uchar> &images, cv::Mat_<int> (*transform)(const cv::Mat_<uchar> &)
);
template stereo::correlation_s<int, int>::correlation_s(int rows, int cols, int depth);
template stereo::correlation_s<int, int>::correlation_s(const stereo::stereo_s<int> &images, int (*distance)(int, int));
template int stereo::correlation_s<int, int>::find_correlation(int i, int j, std::pair<int, int> kernel_size);


template <typename T>
stereo::stereo_s<T>::stereo_s(const std::string &img_left, const std::string &img_right)
{
    cv::Mat lft = imread(img_left, cv::IMREAD_COLOR);
    cv::Mat rgh = imread(img_right, cv::IMREAD_COLOR);

    cvtColor(lft, left, cv::COLOR_BGR2GRAY);
    cvtColor(rgh, right, cv::COLOR_BGR2GRAY);
}

template <typename T>
stereo::stereo_s<T>::stereo_s(const stereo::stereo_s<uchar> &images, cv::Mat_<T> (*transform)(const cv::Mat_<uchar> &))
{
    left = transform(images.left);
    right = transform(images.right);
}

template <typename T>
void stereo::stereo_s<T>::display(const std::pair<std::string, std::string>& window_names,
                                  std::pair<int, int> window_pos,
                                  int displacement) const
{
    cv::imshow(window_names.first, left);
    cv::moveWindow(window_names.first, window_pos.first, window_pos.second);
    cv::imshow(window_names.second, right);
    cv::moveWindow(window_names.second, window_pos.first + left.cols + displacement, window_pos.second);
    cv::waitKey();
}

template <typename T, typename K>
stereo::correlation_s<T, K>::correlation_s(int rows, int cols, int depth) : size({rows, cols, depth})
{
    mat = new K **[size.rows];
    for (int i = 0; i < size.rows; ++i) {
        mat[i] = new K *[size.cols];
        for (int j = 0; j < size.cols; ++j)
            mat[i][j] = new K[size.depth];
    }
}

template <typename T, typename K>
stereo::correlation_s<T, K>::correlation_s(const stereo::stereo_s<T> &images, K (*distance)(T, T)) : correlation_s(images.left.rows, images.left.cols, images.left.cols)
{
    for (int i = 0; i < images.left.rows; ++i)
        for (int j = 0; j < images.left.cols; ++j)
            for (int k = 0; k < images.right.cols; ++k)
                mat[i][j][k] = distance(images.left(i, j), images.right(i, k));
}

template <typename T, typename K>
K &stereo::correlation_s<T, K>::at(int i, int j, int k)
{
    return mat[i][j][k];
}

static inline bool is_inside(std::pair<int, int> size, std::pair<int, int> p)
{
    return (p.first >= 0 && p.first < size.first) && (p.second >= 0 && p.second < size.second);
}

template <typename T, typename K>
int stereo::correlation_s<T, K>::find_correlation(int i, int j, std::pair<int, int> kernel_size)
{
    std::pair<int, int> start_pix = {i - kernel_size.first/2, j - kernel_size.second/2};
    std::pair<int, K> minimum = {INT_MAX, std::numeric_limits<K>::max()};

    for (int k = j; k >= 0; --k) {

        K sum = 0;
        for (int l = 0; l < kernel_size.first; ++l) {
            for (int m = 0; m < kernel_size.second; ++m) {
                std::pair<int, int> curr = {start_pix.first + l, start_pix.second + m};
                if (is_inside({size.rows, size.cols}, curr))
                    sum += at(curr.first, curr.second, k);
                else
                    sum += at(i, j, k);
            }
        }
        if (minimum.second > sum)
            minimum = {k, sum};

    }

    return minimum.first;
}
