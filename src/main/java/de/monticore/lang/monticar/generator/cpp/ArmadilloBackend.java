/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.monticar.generator.MathBackend;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

/**
 */
public class ArmadilloBackend implements MathBackend {
    @Override
    public String getMatrixTypeName() {
        return "mat";
    }

    @Override
    public String getMatrixTypeUnsignedCharName() {
        return "arma::Mat<unsigned char>";
    }

    @Override
    public String getCubeTypeName() {
        return "cube";
    }

    @Override
    public String getCubeUnsignedCharName() {
        return "Cube<unsigned char>";
    }

    @Override
    public String getMatrixInitString(int sizeN, int sizeM) {
        return "(" + sizeN + "," + sizeM + ");\n";
    }

    @Override
    public String getRowVectorTypeName() {
        return "rowvec";
    }

    @Override
    public String getColumnVectorTypeName() {
        return "colvec";
    }

    @Override
    public String getColumnAccessCommandName() {
        return "col";
    }

    @Override
    public String getRowAccessCommandName() {
        return "row";
    }

    @Override
    public String getTubeAccessCommandName() {
        return "tube";
    }

    @Override
    public String getBackendName() {
        return "ArmadilloBackend";
    }

    @Override
    public String getTransposeCommand() {
        return "t";
    }

    @Override
    public String getIncludeHeaderName() {
        return "armadillo";
    }

    @Override
    public String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return getPowerOfString(mathExpressionSymbol, valueListString, ",");
    }

    @Override
    public String getPowerOfString(MathArithmeticExpressionSymbol mathExpressionSymbol, String valueListString, String separator) {
        /*String matrixName = StringValueListExtractorUtil.getElement(valueListString, 0, separator);
        String result = matrixName;
        String powerOfNumber = StringValueListExtractorUtil.getElement(valueListString, 1, separator);
        for (int c = 1; !(c + "").equals(powerOfNumber); ++c) {
            result += "*" + matrixName;
        }*/

        Log.error(mathExpressionSymbol.getTextualRepresentation()+System.lineSeparator()+"Break down power of into smaller multiplications, this is not fully supported by this backend");
        return null;
    }

    @Override
    public String getPowerOfString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        /*String matrixName = StringValueListExtractorUtil.getElement(valueListString, 0);
        String result = matrixName;
        for (int c = 1; !(c + "").equals(StringValueListExtractorUtil.getElement(valueListString, 1)); ++c) {
            result += "*" + matrixName;
        }*/
        Log.error("Break down power of into smaller multiplications, this is not fully supported by this backend");
        return null;
    }

    @Override
    public String getPowerOfEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return "pow" + valueListString;
    }

    @Override
    public String getDivisionEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getLeftExpression(), new ArrayList<>()) + "/" +
                ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getRightExpression(), new ArrayList<>());
    }

    @Override
    public String getMultiplicationEEString(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, String valueListString) {
        return ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getLeftExpression(), new ArrayList<>()) + " % " +
                ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol.getRightExpression(), new ArrayList<>());
    }

    @Override
    public boolean usesZeroBasedIndexing() {
        return true;
    }

    @Override
    public String getWholeNumberRowVectorTypeName() {
        return "irow";
    }

    @Override
    public String getWholeNumberColumnVectorTypeName() {
        return "ivec";
    }

    @Override
    public String getWholeNumberMatrixTypeName() {
        return "imat";
    }

    @Override
    public String getWholeNumberCubeTypeName() {
        return "icube";
    }

    @Override
    public String getMathMatrixColonVectorString(MathMatrixVectorExpressionSymbol symbol) {
        String start = ExecuteMethodGenerator.generateExecuteCode(symbol.getStart(), new ArrayList<>());
        String delta = "1";
        if (symbol.getStep().isPresent())
            delta = ExecuteMethodGenerator.generateExecuteCode(symbol.getStep().get(), new ArrayList<>());
        String end = ExecuteMethodGenerator.generateExecuteCode(symbol.getEnd(), new ArrayList<>());
        return String.format("regspace<rowvec>(%s, %s, %s)", start, delta, end);
    }
}
