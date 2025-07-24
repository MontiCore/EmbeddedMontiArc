/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._cocos;


import de.monticore.lang.math._matrixprops.PropertyChecker;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._ast.ASTMathAssignmentStatement;
import de.monticore.lang.math._matrixprops.MatrixProperties;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Philipp Goerick
 *
 * Matrix Properties Coco
 */

public class MatrixAssignmentCheck implements MathASTMathAssignmentStatementCoCo {
    @Override
    public void check(ASTMathAssignmentStatement assignment) {
        Symbol symbol = assignment.getEnclosingScopeOpt().get()
                .resolve(assignment.getName(),new MathExpressionSymbolKind()).get();
        if (!((MathValueSymbol)symbol).getType().getProperties().isEmpty()) {
            checkMatrixOperation(assignment, (MathValueSymbol) symbol);
        }
    }

    private void checkMatrixOperation(ASTMathAssignmentStatement assignment, MathValueSymbol symbol) {
        MathExpressionSymbol expressionSymbol = ((MathExpressionSymbol)assignment.getExpression().getSymbolOpt().get());
        List<String> expProps = symbol.getType().getProperties();
        ArrayList<MatrixProperties> props = new ArrayList<>();
        MathArithmeticExpressionSymbol arithmeticExpressionSymbol = new MathArithmeticExpressionSymbol();
        arithmeticExpressionSymbol.setLeftExpression(symbol.getValue());
        arithmeticExpressionSymbol.setRightExpression(expressionSymbol);
        props = getMatrixProperties(assignment, expressionSymbol, props, arithmeticExpressionSymbol);
        compareArrays(symbol, expProps, props);
    }

    private ArrayList<MatrixProperties> getMatrixProperties(ASTMathAssignmentStatement assignment, MathExpressionSymbol expressionSymbol, ArrayList<MatrixProperties> props, MathArithmeticExpressionSymbol arithmeticExpressionSymbol) {
        switch (assignment.getMathAssignmentOperator().getOperator()) {
            case "=":{
                props = solveEquation(assignment, expressionSymbol, props);
                break; }
            case "+=": {
                arithmeticExpressionSymbol.setMathOperator("+");
                props = PropertyChecker.checkProps(arithmeticExpressionSymbol);
                break; }
            case "-=": {
                arithmeticExpressionSymbol.setMathOperator("-");
                props = PropertyChecker.checkProps(arithmeticExpressionSymbol);
                break; }
            case "*=": {
                arithmeticExpressionSymbol.setMathOperator("*");
                props = PropertyChecker.checkProps(arithmeticExpressionSymbol);
                break; }
        }
        return props;
    }

    private ArrayList<MatrixProperties> solveEquation(ASTMathAssignmentStatement assignment, MathExpressionSymbol expressionSymbol, ArrayList<MatrixProperties> props) {
        if (expressionSymbol instanceof IArithmeticExpression)
            props = PropertyChecker.checkProps(((IArithmeticExpression) expressionSymbol));
        else if (expressionSymbol instanceof MathMatrixArithmeticValueSymbol)
            props = ((MathMatrixArithmeticValueSymbol) expressionSymbol).getMatrixProperties();
        else if (expressionSymbol instanceof MathNameExpressionSymbol){
            Symbol sym = assignment.getEnclosingScopeOpt().get()
                    .resolve(((MathNameExpressionSymbol) expressionSymbol).getNameToResolveValue(),new MathExpressionSymbolKind()).get();
            props = ((MathValueSymbol) sym).getMatrixProperties();
        }
        return props;
    }

    private void compareArrays(MathValueSymbol symbol, List<String> expProps, ArrayList<MatrixProperties> props) {
        ArrayList<String> props_String = new ArrayList<>();
        for (int i = 0; i < props.size(); i++) props_String.add(props.get(i).toString());

        if (!props_String.containsAll(expProps))
            Log.error("Matrix does not fullfill given properties");
        else{
            symbol.setMatrixProperties(props);
        }
    }
}
