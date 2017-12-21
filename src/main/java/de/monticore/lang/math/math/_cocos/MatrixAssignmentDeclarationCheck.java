/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
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
package de.monticore.lang.math.math._cocos;


import de.monticore.lang.math.math._ast.ASTMathAssignmentDeclarationExpression;
import de.monticore.lang.math.math._matrixprops.MatrixProperties;
import de.monticore.lang.math.math._matrixprops.PropertyChecker;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Philipp Goerick on 07.09.2017.
 *
 * Matrix Properties Coco
 */

public class MatrixAssignmentDeclarationCheck extends AbstChecker implements MathASTMathAssignmentDeclarationExpressionCoCo {
    @Override
    public void check(ASTMathAssignmentDeclarationExpression assignment) {
        if (!assignment.getType().getMatrixProperty().isEmpty()) {
            checkMatrixOperation(assignment);
        }
    }

    private void checkMatrixOperation(ASTMathAssignmentDeclarationExpression assignment) {
        MathExpressionSymbol value = ((MathExpressionSymbol)assignment.getMathExpression().getSymbol().get());
        List<String> expProps = assignment.getType().getMatrixProperty();
        ArrayList<MatrixProperties> props = new ArrayList<>();
        if (assignment.getMathAssignmentOperator().getOperator().get().equals("=")) {
            if (value.isValueExpression()) props = ((MathMatrixArithmeticValueSymbol)value).getMatrixProperties();
            if (value.isArithmeticExpression())
                props = PropertyChecker.checkProps((MathArithmeticExpressionSymbol) value);
            compareArrays(assignment, expProps, props);
        }
    }

    private void compareArrays(ASTMathAssignmentDeclarationExpression assignment, List<String> expProps, ArrayList<MatrixProperties> props) {
        ArrayList<String> props_String = new ArrayList<>();
        for (int i = 0; i < props.size(); i++) props_String.add(props.get(i).toString());
        if (!props_String.containsAll(expProps))
            Log.error("Matrix does not fullfill given properties");
        else{
            Symbol symbol = assignment.getEnclosingScope().get()
                    .resolve(assignment.getName(), new MathExpressionSymbolKind()).get();
            ((MathValueSymbol)symbol).setMatrixProperties(props);
        }
    }
}