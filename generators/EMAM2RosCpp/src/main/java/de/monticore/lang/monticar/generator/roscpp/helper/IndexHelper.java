/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class IndexHelper {
    private IndexHelper() {
    }

    public static List<String> getDimSizesOfMatrixType(ASTCommonMatrixType matrixType) {
        List<String> res = new ArrayList<>();
        for (ASTExpression expression :  matrixType.getDimension().getDimensionList()) {
            if(expression instanceof ASTNameExpression){
                res.add(((ASTNameExpression) expression).getName());
            }else if(expression instanceof ASTNumberExpression){
                Long number = ((ASTNumberExpression)expression).getNumberWithUnit().getNumber().map(Math::round).orElse(null);
                if(number != null){
                    res.add(number.toString());
                }else{
                    Log.error("0xe8035: Could not convert dimension to integer");
                }
            }else{
                Log.error("0x023bd: Dimension must be Name- or NumberExpression!");
            }
        }
        return res;
    }
}
