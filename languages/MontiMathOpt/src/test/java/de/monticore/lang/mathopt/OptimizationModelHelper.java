/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt;

import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.mathopt._ast.ASTMathOptCompilationUnit;
import de.monticore.lang.mathopt._ast.ASTOptimizationStatement;
import de.monticore.lang.mathopt._cocos.MathOptCoCoChecker;
import de.monticore.lang.mathopt._cocos.MathOptCocos;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

import java.util.List;

/**
 * Helper class which provides access to multiple OptimizationSymbols in different models.
 *
 */
public class OptimizationModelHelper extends AbstractMathOptChecker {

    private static OptimizationModelHelper ourInstance = new OptimizationModelHelper();


    public static OptimizationModelHelper getInstance() {
        return ourInstance;
    }

    // optimization expression symbols
    private MathOptimizationStatementSymbol minimizationTestSymbol;
    private MathOptimizationStatementSymbol maximizationTestSymbol;
    private MathOptimizationStatementSymbol lpTestSymbol;
    private MathOptimizationStatementSymbol upperAndLowerBoundTestSymbol;
    private MathOptimizationStatementSymbol forLoopConditionTestSymbol;
    private MathOptimizationStatementSymbol mpcTestSymbol;

    private OptimizationModelHelper() {

    }

    @Override
    protected MathOptCoCoChecker getChecker() {
        return MathOptCocos.createChecker();
    }

    public MathOptimizationStatementSymbol getMathOptimizationStatementSymbolFromTestScript(String pathToModel, int index) {
        // get all math expressions in script
        ASTMathOptCompilationUnit root = loadModel(pathToModel);
        List<ASTStatement> mathExpressions = root.getMathCompilationUnit().getMathScript().getStatementsList();
        // delete non MathOptimizationStatementSymbols
        mathExpressions.removeIf(astMathExpression -> !(astMathExpression instanceof ASTOptimizationStatement));
        return (MathOptimizationStatementSymbol) mathExpressions.get(index).getSymbolOpt().orElse(null);
    }

    public MathOptimizationStatementSymbol getMinimizationTestSymbol() {
        if (minimizationTestSymbol == null)
            minimizationTestSymbol = getMathOptimizationStatementSymbolFromTestScript("src/test/resources/optimization/MinimizationTest.m", 0);
        return minimizationTestSymbol;
    }

    public MathOptimizationStatementSymbol getMaximizationTestSymbol() {
        if (maximizationTestSymbol == null)
            maximizationTestSymbol = getMathOptimizationStatementSymbolFromTestScript("src/test/resources/optimization/MaximizationTest.m", 0);
        return maximizationTestSymbol;
    }

    public MathOptimizationStatementSymbol getLpTestSymbol() {
        if (lpTestSymbol == null)
            lpTestSymbol = getMathOptimizationStatementSymbolFromTestScript("src/test/resources/optimization/LpTest.m", 0);
        return lpTestSymbol;
    }

    public MathOptimizationStatementSymbol getUpperAndLowerBoundTestSymbol() {
        if (upperAndLowerBoundTestSymbol == null) {
            upperAndLowerBoundTestSymbol = getMathOptimizationStatementSymbolFromTestScript("src/test/resources/optimization/UpperAndLowerBoundTest.m", 0);
        }
        return upperAndLowerBoundTestSymbol;
    }

    public MathOptimizationStatementSymbol getForLoopConditionTestSymbol() {
        if (forLoopConditionTestSymbol == null) {
            forLoopConditionTestSymbol = getMathOptimizationStatementSymbolFromTestScript("src/test/resources/optimization/ForLoopConditionTest.m", 0);
        }
        return forLoopConditionTestSymbol;
    }

    public MathOptimizationStatementSymbol getMPCTestSymbol(){
        if (mpcTestSymbol == null) {
            mpcTestSymbol = getMathOptimizationStatementSymbolFromTestScript("src/test/resources/optimization/MPCTest.m", 0);
        }
        return mpcTestSymbol;
    }

}
