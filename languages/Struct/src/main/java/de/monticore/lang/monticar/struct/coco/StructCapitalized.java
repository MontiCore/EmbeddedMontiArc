/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct.coco;

import de.monticore.lang.monticar.struct._ast.ASTStruct;
import de.monticore.lang.monticar.struct._cocos.StructASTStructCoCo;
import de.se_rwth.commons.logging.Log;

public class StructCapitalized implements StructASTStructCoCo {
    @Override
    public void check(ASTStruct node) {
        if (!Character.isUpperCase(node.getName().charAt(0))) {
            Log.error(
                    "Struct names must start with upper-case",
                    node.get_SourcePositionStart()
            );
        }
    }
}
