#ifndef CONVHELPER_H
#define CONVHELPER_H
#include <iostream>
#include "armadillo"
#include <stdarg.h>
#include <initializer_list>
#include <fstream>
using namespace arma;


// convert an OpenCV matrix to Armadillo matrix. NOTE: a copy is made
template <typename T>
arma::Mat<T> to_arma(const cv::Mat_<T>& src) {
    arma::Mat<T> dst(reinterpret_cast<T*>(src.data), src.cols, src.rows);
    //src.copyTo({ src.rows, src.cols, dst.memptr() });
    return dst;
}

// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made
template<typename T>
cv::Mat_<T> to_cvmat(const arma::Mat<T>& src) {
    return cv::Mat_<T>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};
}

// convert an OpenCV multi-channel matrix to Armadillo cube. A copy is made
template <typename T, int NC>
Cube<T> to_armaCube(const cv::Mat_<cv::Vec<T, NC>>& src)
{
    std::vector<cv::Mat_<T>> channels;
    Cube<T> dst(src.cols, src.rows, NC);
    for (int c = 0; c < NC; ++c)
        channels.push_back({ src.rows, src.cols, dst.slice(c).memptr() });
    cv::split(src, channels);
    return dst;
}

// convert an Armadillo cube to OpenCV matrix. NOTE: a copy is made
template <typename T>
cv::Mat to_cvmat(const Cube<T>& src) {
    std::vector<cv::Mat_<T>> channels;
    for (size_t c = 0; c < src.n_slices; ++c) {
        auto* data = const_cast<T*>(src.slice(c).memptr());
        channels.push_back({ int(src.n_cols), int(src.n_rows), data });
    }
    cv::Mat dst;
    cv::merge(channels, dst);
    return dst;
}


#endif