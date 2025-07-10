/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

/**
 * @author Ahmed Diab
 */
public class ConversionHelperSource {

    public static String conversionHelperSourceCode = "#ifndef CONVHELPER_H\n" +
            "#define CONVHELPER_H\n" +
            "#include <iostream>\n" +
            "#include \"armadillo\"\n" +
            "#include <stdarg.h>\n" +
            "#include <initializer_list>\n" +
            "#include <fstream>\n" +
            "using namespace arma;\n" +
            "\n" +
            "\n" +
            "// convert an OpenCV matrix to Armadillo matrix. NOTE: a copy is made\n" +
            "template <typename T>\n" +
            "arma::Mat<T> to_arma(const cv::Mat_<T>& src) {\n" +
            "    arma::Mat<T> dst(reinterpret_cast<T*>(src.data), src.cols, src.rows);\n" +
            "    //src.copyTo({ src.rows, src.cols, dst.memptr() });\n" +
            "    return dst;\n" +
            "}\n" +
            "\n" +
            "// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made\n" +
            "template<typename T>\n" +
            "cv::Mat_<T> to_cvmat(const arma::Mat<T>& src) {\n" +
            "    return cv::Mat_<T>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};\n" +
            "}\n" +
            "\n" +
            "// convert an OpenCV multi-channel matrix to Armadillo cube. A copy is made\n" +
            "template <typename T, int NC>\n" +
            "Cube<T> to_armaCube(const cv::Mat_<cv::Vec<T, NC>>& src)\n" +
            "{\n" +
            "    std::vector<cv::Mat_<T>> channels;\n" +
            "    Cube<T> dst(src.cols, src.rows, NC);\n" +
            "    for (int c = 0; c < NC; ++c)\n" +
            "        channels.push_back({ src.rows, src.cols, dst.slice(c).memptr() });\n" +
            "    cv::split(src, channels);\n" +
            "    return dst;\n" +
            "}\n" +
            "\n" +
            "// convert an Armadillo cube to OpenCV matrix. NOTE: a copy is made\n" +
            "template <typename T>\n" +
            "cv::Mat to_cvmat(const Cube<T>& src) {\n" +
            "    std::vector<cv::Mat_<T>> channels;\n" +
            "    for (size_t c = 0; c < src.n_slices; ++c) {\n" +
            "        auto* data = const_cast<T*>(src.slice(c).memptr());\n" +
            "        channels.push_back({ int(src.n_cols), int(src.n_rows), data });\n" +
            "    }\n" +
            "    cv::Mat dst;\n" +
            "    cv::merge(channels, dst);\n" +
            "    return dst;\n" +
            "}\n" +
            "\n" +
            "\n" +
            "#endif";
}
