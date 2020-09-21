/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.enumlang.coco;

import de.monticore.lang.monticar.enumlang._ast.ASTEnumConstantDeclaration;
import de.monticore.lang.monticar.enumlang._ast.ASTEnumDeclaration;
import de.monticore.lang.monticar.enumlang._cocos.EnumLangASTEnumDeclarationCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class EnumConstantUnique implements EnumLangASTEnumDeclarationCoCo {
    @Override
    public void check(ASTEnumDeclaration node) {
        Set<String> names = new HashSet<>();
        for (ASTEnumConstantDeclaration ecd : node.getEnumConstantDeclarationList()) {
            if (!names.add(ecd.getName())) {
                Log.error(
                        "Enum constants must be unique",
                        node.get_SourcePositionStart()
                );
            }
        }
    }
}
