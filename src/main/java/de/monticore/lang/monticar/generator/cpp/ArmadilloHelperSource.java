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
            "\n" +
            "static mat getKMeansClusters(mat A, int k){\n" +
            "mat clusters;\n" +
            "kmeans(clusters,A.t(),k,random_subset,20,true);\n" +
            "printf(\"cluster centroid calculation done\\n\");\n" +
            "std::ofstream myfile;\n" +
            "     myfile.open(\"data after cluster.txt\");\n" +
            "     myfile << A;\n" +
            "     myfile.close();\n" +
            "\t \n" +
            "\t std::ofstream myfile2;\n" +
            "     myfile2.open(\"cluster centroids.txt\");\n" +
            "     myfile2 << clusters;\n" +
            "     myfile2.close();\n" +
            "mat indexedData=getKMeansClustersIndexData(A.t(), clusters);\n" +
            "\n" +
            "std::ofstream myfile3;\n" +
            "     myfile3.open(\"data after index.txt\");\n" +
            "     myfile3 << indexedData;\n" +
            "     myfile3.close();\n" +
            "\t \n" +
            "return indexedData;\n" +
            "}\n" +
            "\n" +
            "static mat getKMeansClustersIndexData(mat A, mat centroids){\n" +
            "\tmat result=mat(A.n_cols, 1);\n" +
            "\tfor(int i=0;i<A.n_cols;++i){\n" +
            "\t\tresult(i, 0) = getIndexForClusterCentroids(A, i, centroids);\n" +
            "\t}\n" +
            "\treturn result;\n" +
            "}\n" +
            "\n" +
            "static int getIndexForClusterCentroids(mat A, int colIndex, mat centroids){\n" +
            "\tint index=1;\n" +
            "\tdouble lowestDistance=getEuclideanDistance(A, colIndex, centroids, 0);\n" +
            "\tfor(int i=1;i<centroids.n_cols;++i){\n" +
            "\t\tdouble curDistance=getEuclideanDistance(A, colIndex, centroids, i);\n" +
            "\t\tif(curDistance<lowestDistance){\n" +
            "\t\t\tlowestDistance=curDistance;\n" +
            "\t\t\tindex=i+1;\n" +
            "\t\t}\n" +
            "\t}\n" +
            "\treturn index;\n" +
            "}\n" +
            "\n" +
            "static double getEuclideanDistance(mat A, int colIndexA, mat B, int colIndexB){\n" +
            "\tdouble distance=0;\n" +
            "\tfor(int i=0;i<A.n_rows;++i){\n" +
            "\t\tdouble elementA=A(i,colIndexA);\n" +
            "\t\tdouble elementB=B(i,colIndexB);\n" +
            "\t\tdouble diff=elementA-elementB;\n" +
            "\t\tdistance+=diff*diff;\n" +
            "\t}\n" +
            "\treturn sqrt(distance);\n" +
            "}\n" +
            "\n" +
            "static mat getSqrtMat(mat A){\n" +
            "cx_mat result=sqrtmat(A);\n" +
            "return real(result);\n" +
            "}\n" +
            "};\n" +
            "#endif\n";
}
