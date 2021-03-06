/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._cocos;


import de.monticore.lang.math._ast.ASTMathAssignmentDeclarationStatement;
import de.monticore.lang.math._matrixprops.MatrixProperties;
import de.monticore.lang.math._matrixprops.PropertyChecker;
import de.monticore.lang.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbolKind;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Philipp Goerick on 07.09.2017.
 *
 * Matrix Properties Coco
 */

public class MatrixAssignmentDeclarationCheck implements MathASTMathAssignmentDeclarationStatementCoCo {
    @Override
    public void check(ASTMathAssignmentDeclarationStatement assignment) {
        if (!assignment.getType().getMatrixPropertyList().isEmpty()) {
            checkMatrixOperation(assignment);
        }
    }

    private void checkMatrixOperation(ASTMathAssignmentDeclarationStatement assignment) {
        MathExpressionSymbol value = ((MathExpressionSymbol) assignment.getExpression().getSymbolOpt().get());
        List<String> expProps = assignment.getType().getMatrixPropertyList();
        ArrayList<MatrixProperties> props = new ArrayList<>();
        if (assignment.getMathAssignmentOperator().getOperator().equals("=")) {
            if (value.isValueExpression()) props = ((MathMatrixArithmeticValueSymbol) value).getMatrixProperties();
            if (value.isArithmeticExpression() || value instanceof MathMatrixArithmeticExpressionSymbol)
                props = PropertyChecker.checkProps((IArithmeticExpression) value);
            compareArrays(assignment, expProps, props);
        }
    }

    private void compareArrays(ASTMathAssignmentDeclarationStatement assignment, List<String> expProps, ArrayList<MatrixProperties> props) {
        ArrayList<String> props_String = new ArrayList<>();
        for (int i = 0; i < props.size(); i++) props_String.add(props.get(i).toString());
        if (!props_String.containsAll(expProps))
            Log.error("Matrix does not fullfill given properties");
        else {
            Symbol symbol = assignment.getEnclosingScopeOpt().get()
                    .resolve(assignment.getName(), new MathExpressionSymbolKind()).get();
            ((MathValueSymbol) symbol).setMatrixProperties(props);
        }
    }
}
