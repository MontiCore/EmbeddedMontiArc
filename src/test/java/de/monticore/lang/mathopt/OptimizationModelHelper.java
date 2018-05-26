/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.mathopt;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.mathopt._ast.ASTMathOptCompilationUnit;
import de.monticore.lang.mathopt._ast.ASTOptimizationExpression;
import de.monticore.lang.mathopt._cocos.MathOptCoCoChecker;
import de.monticore.lang.mathopt._cocos.MathOptCocos;
import de.monticore.lang.mathopt._symboltable.MathOptimizationExpressionSymbol;

import java.util.List;

/**
 * Helper class which provides access to multiple OptimizationSymbols in different models.
 * @author Christoph Richter
 */
public class OptimizationModelHelper extends AbstractMathOptChecker {

    private static OptimizationModelHelper ourInstance = new OptimizationModelHelper();

    public static OptimizationModelHelper getInstance() {
        return ourInstance;
    }

    // optimization expression symbols
    private MathOptimizationExpressionSymbol minimizationTestSymbol;
    private MathOptimizationExpressionSymbol maximizationTestSymbol;
    private MathOptimizationExpressionSymbol lpTestSymbol;
    private MathOptimizationExpressionSymbol upperAndLowerBoundTestSymbol;
    private MathOptimizationExpressionSymbol forLoopConditionTestSymbol;
    private MathOptimizationExpressionSymbol existingOptimizationVarScalar;
    private MathOptimizationExpressionSymbol existingOptimizationVarMatrix;
    private MathOptimizationExpressionSymbol existingOptimizationVarSubstituted;

    private OptimizationModelHelper() {
        try {
            loadOptimizationSymbols();
        } finally {
        }
    }

    @Override
    protected MathOptCoCoChecker getChecker() {
        return MathOptCocos.createChecker();
    }

    public MathOptimizationExpressionSymbol getMathOptimizationExpressionSymbolFromTestScript(String pathToModel, int index) {
        // get all math expressions in script
        ASTMathOptCompilationUnit root = loadModel(pathToModel);
        List<ASTStatement> mathExpressions = root.getMathCompilationUnit().getMathScript().getStatements().getStatementList();
        // delete non MathOptimizationExpressionSymbols
        mathExpressions.removeIf(astMathExpression -> !(astMathExpression instanceof ASTOptimizationExpression));
        return (MathOptimizationExpressionSymbol) mathExpressions.get(index).getSymbolOpt().orElse(null);
    }

    private void loadOptimizationSymbols() {
        minimizationTestSymbol = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/MinimizationTest.m", 0);
        maximizationTestSymbol = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/MaximizationTest.m", 0);
        lpTestSymbol = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/LpTest.m", 0);
        upperAndLowerBoundTestSymbol = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/UpperAndLowerBoundTest.m", 0);
        forLoopConditionTestSymbol = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/ForLoopConditionTest.m", 0);
        existingOptimizationVarScalar = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/ExistingOptimizationVariable.m", 0);
        existingOptimizationVarMatrix = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/ExistingOptimizationVariable.m", 1);
        existingOptimizationVarSubstituted = getMathOptimizationExpressionSymbolFromTestScript("src/test/resources/optimization/ExistingOptimizationVariable.m", 2);
    }

    public MathOptimizationExpressionSymbol getMinimizationTestSymbol() {
        return minimizationTestSymbol;
    }

    public MathOptimizationExpressionSymbol getMaximizationTestSymbol() {
        return maximizationTestSymbol;
    }

    public MathOptimizationExpressionSymbol getLpTestSymbol() {
        return lpTestSymbol;
    }

    public MathOptimizationExpressionSymbol getUpperAndLowerBoundTestSymbol() {
        return upperAndLowerBoundTestSymbol;
    }

    public MathOptimizationExpressionSymbol getForLoopConditionTestSymbol() {
        return forLoopConditionTestSymbol;
    }

    public MathOptimizationExpressionSymbol getExistingOptimizationVarScalar() {
        return existingOptimizationVarScalar;
    }

    public MathOptimizationExpressionSymbol getExistingOptimizationVarMatrix() {
        return existingOptimizationVarMatrix;
    }

    public MathOptimizationExpressionSymbol getExistingOptimizationVarSubstituted() {
        return existingOptimizationVarSubstituted;
    }

    protected void setMinimizationTestSymbol(MathOptimizationExpressionSymbol minimizationTestSymbol) {
        this.minimizationTestSymbol = minimizationTestSymbol;
    }

    protected void setMaximizationTestSymbol(MathOptimizationExpressionSymbol maximizationTestSymbol) {
        this.maximizationTestSymbol = maximizationTestSymbol;
    }

    protected void setLpTestSymbol(MathOptimizationExpressionSymbol lpTestSymbol) {
        this.lpTestSymbol = lpTestSymbol;
    }

    protected void setUpperAndLowerBoundTestSymbol(MathOptimizationExpressionSymbol upperAndLowerBoundTestSymbol) {
        this.upperAndLowerBoundTestSymbol = upperAndLowerBoundTestSymbol;
    }

    protected void setForLoopConditionTestSymbol(MathOptimizationExpressionSymbol forLoopConditionTestSymbol) {
        this.forLoopConditionTestSymbol = forLoopConditionTestSymbol;
    }

    protected void setExistingOptimizationVarScalar(MathOptimizationExpressionSymbol existingOptimizationVarScalar) {
        this.existingOptimizationVarScalar = existingOptimizationVarScalar;
    }

    protected void setExistingOptimizationVarMatrix(MathOptimizationExpressionSymbol existingOptimizationVarMatrix) {
        this.existingOptimizationVarMatrix = existingOptimizationVarMatrix;
    }

    protected void setExistingOptimizationVarSubstituted(MathOptimizationExpressionSymbol existingOptimizationVarSubstituted) {
        this.existingOptimizationVarSubstituted = existingOptimizationVarSubstituted;
    }
}
