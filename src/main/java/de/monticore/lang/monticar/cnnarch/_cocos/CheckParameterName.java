/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._ast.ASTLayerParameter;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchParameter;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class CheckParameterName implements CNNArchASTArchParameterCoCo {

    Set<String> parameterNames = new HashSet<>();


    @Override
    public void check(ASTArchParameter node) {
        checkForIllegalNames(node);
        checkForDuplicates(node);
    }

    private void checkForIllegalNames(ASTArchParameter node){
        String name = node.getName();
        if (name.isEmpty() || !Character.isLowerCase(name.codePointAt(0))){
            Log.error("0" + ErrorCodes.ILLEGAL_NAME + " Illegal name: " + name +
                            ". All new parameter and method names have to start with a lowercase letter. "
                    , node.get_SourcePositionStart());
        }
        else if (name.equals(AllPredefinedVariables.TRUE_NAME) || name.equals(AllPredefinedVariables.FALSE_NAME)){
            Log.error("0" + ErrorCodes.ILLEGAL_NAME + " Illegal name: " + name +
                            ". No parameter can be named 'true' or 'false'"
                    , node.get_SourcePositionStart());
        }
        else if (name.equals(AllPredefinedVariables.CONDITIONAL_ARG_NAME.toLowerCase())){
            Log.error("0" + ErrorCodes.ILLEGAL_NAME + " Illegal name: " + name +
                            ". No parameter can be named 'if'"
                    , node.get_SourcePositionStart());
        }
    }

    private void checkForDuplicates(ASTArchParameter node){
        String name = node.getName();
        if (parameterNames.contains(name)){
            if (node instanceof ASTLayerParameter){
                Collection<Symbol> allParametersWithSameName = node.getEnclosingScopeOpt().get().getLocalSymbols().get(name);
                if (allParametersWithSameName.size() > 1){
                    duplicationError(node);
                }
            }
            else {
                duplicationError(node);
            }
        }
        else{
            parameterNames.add(name);
        }
    }

    private void duplicationError(ASTArchParameter node){
        Log.error("0" + ErrorCodes.DUPLICATED_NAME + " Duplicated parameter name. " +
                        "The name '" + node.getName() + "' is already used."
                , node.get_SourcePositionStart());
    }

}
