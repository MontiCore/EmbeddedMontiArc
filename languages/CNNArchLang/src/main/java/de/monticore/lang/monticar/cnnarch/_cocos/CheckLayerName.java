/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._ast.ASTLayerDeclaration;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class CheckLayerName implements CNNArchASTLayerDeclarationCoCo {

    Set<String> methodNames = new HashSet<>();

    @Override
    public void check(ASTLayerDeclaration node) {
        String name = node.getName();
        if (name.isEmpty() || !Character.isLowerCase(name.codePointAt(0))){
            Log.error("0" + ErrorCodes.ILLEGAL_NAME + " Illegal name: " + name +
                            ". All new variable and method names have to start with a lowercase letter. "
                    , node.get_SourcePositionStart());
        }

        if (methodNames.contains(name)){
            Log.error("0" + ErrorCodes.DUPLICATED_NAME + " Duplicated method name. " +
                            "The name '" + name + "' is already used."
                    , node.get_SourcePositionStart());
        }
        else {
            methodNames.add(name);
        }
    }

}
