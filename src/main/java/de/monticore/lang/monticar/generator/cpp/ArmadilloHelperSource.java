package de.monticore.lang.monticar.generator.cpp;

/**
 * @author Sascha Schneiders
 */
public class ArmadilloHelperSource {
    public static String armadilloHelperSourceCode = "#ifndef HELPERA_H\n" +
            "#define HELPERA_H\n" +
            "#define _GLIBCXX_USE_CXX11_ABI 0\n" +
            "#include <iostream>\n" +
            "#include \"armadillo.h\"\n" +
            "#include <stdarg.h>\n" +
            "#include <initializer_list>\n" +
            "using namespace arma;\n" +
            "class HelperA{\n" +
            "public:\n" +
            "static mat getEigenVectors(mat A){\n" +
            "vec eigenValues;\n" +
            "mat eigenVectors;\n" +
            "eig_sym(eigenValues,eigenVectors,A);\n" +
            "return eigenVectors;\n" +
            "}\n" +
            "static vec getEigenValues(mat A){\n" +
            "vec eigenValues;\n" +
            "mat eigenVectors;\n" +
            "eig_sym(eigenValues,eigenVectors,A);\n" +
            "return eigenValues;\n" +
            "}\n" +
            "static mat getKMeansClusters(mat A, int k){\n" +
            "mat clusters;\n" +
            "kmeans(clusters,A,k,random_subset,10,true);\n" +
            "return clusters;\n" +
            "}\n" +
            "\n" +
            "static mat getSqrtMat(mat A){\n" +
            "cx_mat result=sqrtmat(A);\n" +
            "return real(result);\n" +
            "}\n" +
            "};\n" +
            "#endif\n";
}
