/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.enumlang.coco;

import de.monticore.lang.monticar.enumlang._ast.ASTEnumConstantDeclaration;
import de.monticore.lang.monticar.enumlang._cocos.EnumLangASTEnumConstantDeclarationCoCo;
import de.se_rwth.commons.logging.Log;

public class EnumConstantAllCaps implements EnumLangASTEnumConstantDeclarationCoCo {
    @Override
    public void check(ASTEnumConstantDeclaration node) {
        for (char c : node.getName().toCharArray()) {
            if (Character.isLetter(c) && !Character.isUpperCase(c)) {
                Log.error(
                        "Letters in names of enum constants must be all capitalized",
                        node.get_SourcePositionStart()
                );
            }
        }
    }
}
