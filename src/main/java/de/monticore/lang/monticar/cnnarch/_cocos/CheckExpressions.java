/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.helper.Utils;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class CheckExpressions extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchExpressionSymbol sym) {
        if (sym instanceof ArchSimpleExpressionSymbol){
            checkNamesInExpression((ArchSimpleExpressionSymbol) sym);
        }
    }

    public void checkNamesInExpression(ArchSimpleExpressionSymbol expression) {
        if (expression.getMathExpression().isPresent()){
            MathExpressionSymbol mathExpression = expression.getMathExpression().get();

            for (MathExpressionSymbol subMathExp : Utils.createSubExpressionList(mathExpression)){
                if (subMathExp instanceof MathNameExpressionSymbol){
                    String name = ((MathNameExpressionSymbol) subMathExp).getNameToAccess();
                    Collection<ParameterSymbol> parameterCollection = expression.getEnclosingScope().resolveMany(name, ParameterSymbol.KIND);

                    if (parameterCollection.isEmpty()){
                        Log.error("0" + ErrorCodes.UNKNOWN_PARAMETER_NAME + " Unknown parameter name. " +
                                "The parameter '" + name + "' does not exist. "
                                , subMathExp.getSourcePosition());
                    }
                }
            }
        }
    }
}
