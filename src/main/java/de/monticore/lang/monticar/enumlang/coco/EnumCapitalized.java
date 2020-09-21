/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.enumlang.coco;

import de.monticore.lang.monticar.enumlang._ast.ASTEnumDeclaration;
import de.monticore.lang.monticar.enumlang._cocos.EnumLangASTEnumDeclarationCoCo;
import de.se_rwth.commons.logging.Log;

public class EnumCapitalized implements EnumLangASTEnumDeclarationCoCo {

    @Override
    public void check(ASTEnumDeclaration node) {
        if (!Character.isUpperCase(node.getName().charAt(0))) {
            Log.error(
                    "Enum names must start with upper-case",
                    node.get_SourcePositionStart()
            );
        }
    }
}
