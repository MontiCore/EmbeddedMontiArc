package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.monticar.generator.MathBackend;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

/**
 * @author Sascha Schneiders
 */
public class ArmadilloBackend implements MathBackend {
    @Override
    public String getMatrixTypeName() {
        return "mat";
    }

    @Override
    public String getCubeTypeName() {
        return "cube";
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
        Log.error("Break down power of into smaller multiplications, this is not fully supported by this backend");
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
    public boolean usesZeroBasedIndexing() {
        return true;
    }
}
