/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

/**
 */
public class ArmadilloHelperSource {

    public static String armadilloHelperSourceCode = "#ifndef HELPERA_H\n" +
            "#define HELPERA_H\n" +
            "#include <iostream>\n" +
            "#include \"armadillo\"\n" +
            "#include <stdarg.h>\n" +
            "#include <initializer_list>\n" +
            "#include <fstream>\n" +
            "using namespace arma;\n" +
            "#ifndef _FILESTRING_CONVERSION___A\n" +
            "#define _FILESTRING_CONVERSION___A\n" +
            "void toFileString(std::ofstream& myfile, mat A){\n" +
            "    myfile << \"[\";\n" +
            "    for (int i = 0; i < A.n_rows; i++){\n" +
            "        for (int j = 0; j < A.n_cols; j++){\n" +
            "            myfile << A(i,j);\n" +
            "            if(j + 1 < A.n_cols){\n" +
            "                myfile << \", \";\n" +
            "            }\n" +
            "        }\n" +
            "        if(i + 1 < A.n_rows){\n" +
            "            myfile << \";\";\n" +
            "        }\n" +
            "    }\n" +
            "    myfile << \"]\";\n" +
            "}\n" +
            "void toFileString(std::ofstream& myfile, double A){\n" +
            "    myfile << A;\n" +
            "}\n" +
            "void toFileString(std::ofstream& myfile, float A){\n" +
            "    myfile << A;\n" +
            "}\n" +
            "void toFileString(std::ofstream& myfile, int A){\n" +
            "    myfile << A;\n" +
            "}\n" +
            "void toFileString(std::ofstream& myfile, bool A){\n" +
            "    myfile << A;\n" +
            "}\n" +
            "bool Is_close(mat& X, mat& Y, double tol)\n" +
            "{\n" +
            "    // abs returns a mat type then max checks columns and returns a row_vec\n" +
            "    // max used again will return the biggest element in the row_vec\n" +
            "    bool close(false);\n" +
            "    if(arma::max(arma::max(arma::abs(X-Y))) < tol)\n" +
            "    {\n" +
            "        close = true;\n" +
            "    }\n" +
            "    return close;\n" +
            "}\n" +
            "#endif\n" +
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
            "/*printf(\"cluster centroid calculation done\\n\");\n" +
            "std::ofstream myfile;\n" +
            "     myfile.open(\"data after cluster.txt\");\n" +
            "     myfile << A;\n" +
            "     myfile.close();\n" +
            "\t \n" +
            "\t std::ofstream myfile2;\n" +
            "     myfile2.open(\"cluster centroids.txt\");\n" +
            "     myfile2 << clusters;\n" +
            "     myfile2.close();*/\n" +
            "mat indexedData=getKMeansClustersIndexData(A.t(), clusters);\n" +
            "\n" +
            "/*std::ofstream myfile3;\n" +
            "     myfile3.open(\"data after index.txt\");\n" +
            "     myfile3 << indexedData;\n" +
            "     myfile3.close();\n" +
            "\t */\n" +
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
            "    cx_mat result=sqrtmat(A);\n" +
            "    return real(result);\n" +
            "}\n" +
            "\n" +
            "static mat getSqrtMatDiag(mat A){\n" +
            "for(int i=0;i<A.n_rows;++i){\n" +
            "    double curVal = A(i,i);\n" +
            "    A(i,i) = sqrt(curVal);\n" +
            "}\n" +
            "return A;\n" +
            "}\n" +
            "\n" +
            "static mat invertDiagMatrix(mat A){\n" +
            "for(int i=0;i<A.n_rows;++i){\n" +
            "    double curVal = A(i,i);\n" +
            "    A(i,i) = 1/curVal;\n" +
            "}\n" +
            "return A;\n" +
            "}\n" +
            "};\n" +
            "#endif\n";
}
