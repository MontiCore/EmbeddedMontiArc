/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.util.Arrays;

/**
 */
public class Helper {
    public static MathStatementsSymbol getMathStatementsSymbolFor(EMAComponentInstanceSymbol instanceSymbol, Scope symtab) {
        String resolveName = "MathStatements";
        MathStatementsSymbol mathSymbol = instanceSymbol.getSpannedScope().<MathStatementsSymbol>resolve(resolveName, MathStatementsSymbol.KIND).orElse(null);

        if (mathSymbol == null) {
            resolveName = instanceSymbol.getPackageName() + "." + StringUtils.capitalize(instanceSymbol.getName()) + ".MathStatements";
            mathSymbol = symtab.<MathStatementsSymbol>resolve(resolveName, MathStatementsSymbol.KIND).orElse(null);
        }

        if (mathSymbol == null && instanceSymbol.getComponentType() != null) {
            EMAComponentSymbol symbol = instanceSymbol.getComponentType().getReferencedSymbol();
            resolveName = symbol.getPackageName() + "." + symbol.getName() + ".MathStatements";
            mathSymbol = symtab.<MathStatementsSymbol>resolve(resolveName, MathStatementsSymbol.KIND).orElse(null);
        }

        if (mathSymbol != null)
            Log.info(mathSymbol.toString(), "MathSymbol:");
        else
            Log.info("Could not resolve " + resolveName, "MathSymbol:");
        return mathSymbol;
    }

    public static String convertCubeToString(double[][][] cube) {
        double[][] matrix = cube[0];
        String str = "[";
        for (int k = 0; k < cube.length; k++) {
            str += "[";
            for (int i = 0; i < matrix.length; i++) {
                for (int j = 0; j < matrix[i].length; j++) {
                    str += " " + matrix[i][j];
                    if (j < matrix[i].length - 1) {
                        str += ",";
                    }
                }
                if (i < matrix.length - 1) {
                    str += ";";
                }
            }
            str += "]";
        }

        str += "]";
        return str;
    }

    public static double[][][] translateToCube(double[] source, int[] shape) {
        assert(shape.length == 3);
        double[][][] cubeMatrix = new double[shape[0]][shape[2]][shape[1]]; // slices(channels), columns(width), rows(height)
        final int matrixSize = shape[1] * shape[2];
        long[] matrixShape = new long[]{shape[2], shape[1]};
        int startPos = 0;
        int endPos = matrixSize;

        for (int i = 0; i < shape[0]; i++) {
            double[] matrixSource = Arrays.copyOfRange(source, startPos, endPos);
            cubeMatrix[i] = translateToMat(matrixSource, matrixShape);
            startPos = endPos;
            endPos += matrixSize;
        }
        return cubeMatrix;
    }

    private static double[][] translateToMat(double[] source, long[] shape) { // shape {col, row}
        assert(shape.length == 2);
        double[][] matrix = new double[(int) shape[1]][(int) shape[0]];
        int startPos = 0;
        int endPos = (int) shape[1];
        final int[] columnShape = new int[]{(int) shape[1]};
        for (int i = 0; i < shape[0]; i++) {  // for each column
            double[] colSource = Arrays.copyOfRange(source, startPos, endPos);
            for (int j = 0; j < shape[1]; j++) {
                try {
                    matrix[j][i] = translateToCol(colSource, columnShape)[j];

                } catch (ArrayIndexOutOfBoundsException e) {
                    Log.error("hahah");
                }
            }
            startPos = endPos;
            endPos += shape[1];
        }
        return matrix;
    }

    private static double[] translateToCol(double[] source, int[] shape) {
        assert shape.length == 1;
        double[] column = new double[shape[0]];
        for (int i = 0; i < source.length; i++) {
            column[i] = source[i];
        }
        return column;
    }
}
