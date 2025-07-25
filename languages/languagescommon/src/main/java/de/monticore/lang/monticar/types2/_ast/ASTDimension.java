/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.types2._ast;

import de.monticore.expressionsbasis._ast.ASTExpression;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class ASTDimension extends ASTDimensionTOP {

    public ASTDimension() {
    }

    public ASTDimension(List<ASTExpression> matrixDims, Optional<ASTExpression> vecDim) {
        super(matrixDims, vecDim);
    }

    public List<ASTExpression> getDimensionList() {
        List<ASTExpression> result = new ArrayList<>();
        result.addAll(getMatrixDimList());
        if (getVecDimOpt().isPresent())
            result.add(getVecDim());
        return result;
    }
}
