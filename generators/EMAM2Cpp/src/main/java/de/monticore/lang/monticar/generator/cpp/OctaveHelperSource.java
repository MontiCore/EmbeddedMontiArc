/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

/**
 */
public class OctaveHelperSource {
    public static String octaveHelperSourceCode = "#ifndef HELPER_H\n" +
            "#define HELPER_H\n" +
            "#include <iostream>\n" +
            "#include <octave/oct.h>\n" +
            "#include <octave/octave.h>\n" +
            "#include <octave/parse.h>\n" +
            "#include <octave/interpreter.h>\n" +
            "#include <stdarg.h>\n" +
            "#include <initializer_list>\n" +
            "#include \"octave/builtin-defun-decls.h\"\n" +
            "class Helper\n" +
            "{\n" +
            "public:\n" +
            "    static void init()\n" +
            "    {\n" +
            "        string_vector argv(2);\n" +
            "        argv(0) = \"embedded\";\n" +
            "        argv(1) = \"-q\";\n" +
            "        octave_main(2, argv.c_str_vec(), 1);\n" +
            "        //octave_debug=1;\n" +
            "        //feval (\"pkg\", ovl (\"load\", \"all\"), 0);\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(double a)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(Matrix a)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(RowVector a)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(ColumnVector a)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(double a, double b)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "        in(1) = b;\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(std::initializer_list<double> args)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        int counter = 0;\n" +
            "        for(double element : args) {\n" +
            "            in(counter) = octave_value(element);\n" +
            "            ++counter;\n" +
            "        }\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(Matrix a, double b)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "        in(1) = b;\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(RowVector a, double b)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "        in(1) = b;\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list convertToOctaveValueList(ColumnVector a, double b)\n" +
            "    {\n" +
            "        octave_value_list in;\n" +
            "        in(0) = a;\n" +
            "        in(1) = b;\n" +
            "\n" +
            "        return in;\n" +
            "    }\n" +
            "\n" +
            "    static octave_value_list callOctaveFunction(octave_value_list in, std::string functionName,int argsOut)\n" +
            "    {\n" +
            "\n" +
            "        /*octave_idx_type n = 2;\n" +
            "        octave_value_list in;\n" +
            "\n" +
            "        for(octave_idx_type i = 0; i < n; i++)\n" +
            "            in(i) = octave_value(5 * (i + 2));\n" +
            "\n" +
            "        octave_value_list out = feval(\"gcd\", in, 1);\n" +
            "\n" +
            "        if(!error_state && out.length() > 0)\n" +
            "            std::cout << \"GCD of [\" << in(0).int_value() << \", \" << in(1).int_value() << \"] is \" << out(0).int_value()\n" +
            "                      << std::endl;\n" +
            "        else\n" +
            "            std::cout << \"invalid\\n\";\n" +
            "\n" +
            "        clean_up_and_exit(0);*/\n" +
            "  /*      if(functionName==\"eigs\")\n" +
            "            return feval(functionName, in, 2);\n" +
            "        else if(functionName==\"kmeans\")\n" +
            "            return feval(functionName, in, 2);\n" +
            "*/\n" +
            "        return feval(functionName, in, argsOut);\n" +
            "    }\n" +
            "\n" +
            "    static int callOctaveFunctionIntFirstResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        // printf(\"callOctaveFunctionIntFirstResult pre return functionName: %s\\n\",functionName.c_str());\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(0).int_value();\n" +
            "    }\n" +
            "\n" +
            "    static double callOctaveFunctionDoubleFirstResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        // printf(\"callOctaveFunctionDoubleFirstResult pre return functionName: %s\\n\",functionName.c_str());\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(0).double_value();\n" +
            "    }\n" +
            "\n" +
            "    static Matrix callOctaveFunctionMatrixFirstResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(0).matrix_value();\n" +
            "    }\n" +
            "    static ColumnVector callOctaveFunctionColumnVectorFirstResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        printf(\"pre Call %s\\n\", functionName.c_str());\n" +
            "        try {\n" +
            "            in=octave_value_list();\n" +
            "            octave_value_list list = callOctaveFunction(in, functionName,argsOut);\n" +
            "            printf(\"post Call %s\\n\", functionName.c_str());\n" +
            "            return list(0).array_value().as_column();\n" +
            "        } catch(const std::exception& e) {\n" +
            "            printf(\"%s\\n\", e.what());\n" +
            "        }\n" +
            "        return ColumnVector();\n" +
            "    }\n" +
            "\n" +
            "    static RowVector callOctaveFunctionRowVectorFirstResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(0).array_value().as_row();\n" +
            "    }\n" +
            "\n" +
            "    static int callOctaveFunctionIntSecondResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(1).int_value();\n" +
            "    }\n" +
            "\n" +
            "    static double callOctaveFunctionDoubleSecondResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(1).double_value();\n" +
            "    }\n" +
            "\n" +
            "    static Matrix callOctaveFunctionMatrixSecondResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(1).matrix_value();\n" +
            "    }\n" +
            "    static ColumnVector callOctaveFunctionColumnVectorSecondResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(1).array_value().as_column();\n" +
            "    }\n" +
            "\n" +
            "    static RowVector callOctaveFunctionRowVectorSecondResult(octave_value_list in, std::string functionName, int argsOut)\n" +
            "    {\n" +
            "        return callOctaveFunction(in, functionName,argsOut)(1).array_value().as_row();\n" +
            "    }\n" +
            "    \n" +
            "    static Matrix getMatrixFromOctaveListFirstResult(octave_value_list list){\n" +
            "        return list(0).matrix_value();\n" +
            "    }\n" +
            "    \n" +
            "    \n" +
            "    static RowVector getRowVectorFromOctaveListFirstResult(octave_value_list list){\n" +
            "        return list(0).array_value().as_row();\n" +
            "    }\n" +
            "    \n" +
            "    \n" +
            "    static ColumnVector getColumnVectorFromOctaveListFirstResult(octave_value_list list){\n" +
            "        return list(0).array_value().as_column();\n" +
            "    }\n" +
            "    \n" +
            "    \n" +
            "    static double getDoubleFromOctaveListFirstResult(octave_value_list list){\n" +
            "        return list(0).double_value();\n" +
            "    }\n" +
            "    \n" +
            "    \n" +
            "    static int getIntFromOctaveListFirstResult(octave_value_list list){\n" +
            "        return list(0).int_value();\n" +
            "    }\n" +
            "\n" +
            "    static Matrix getSqrtMatrixDiag(Matrix A){\n" +
            "        int rows = Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(A),0));\n" +
            "        for(int i=0;i<rows;++i){\n" +
            "            double curVal = A(i,i);\n" +
            "            A(i,i) = sqrt(curVal);\n" +
            "        }\n" +
            "        return A;\n" +
            "    }\n" +
            "\n" +
            "    static Matrix invertDiagMatrix(Matrix A){\n" +
            "        int rows = Helper::getDoubleFromOctaveListFirstResult(Fsize(Helper::convertToOctaveValueList(A),0));\n" +
            "    for(int i=0;i<rows;++i){\n" +
            "        double curVal = A(i,i);\n" +
            "        A(i,i) = 1/curVal;\n" +
            "    }\n" +
            "    return A;\n" +
            "    }\n" +
            "};\n" +
            "\n" +
            "#endif // HELPER_H";
}
