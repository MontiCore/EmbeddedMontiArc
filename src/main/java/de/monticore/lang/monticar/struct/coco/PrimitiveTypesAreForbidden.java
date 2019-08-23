/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct.coco;

import de.monticore.types.types._ast.ASTPrimitiveType;
import de.monticore.types.types._cocos.TypesASTPrimitiveTypeCoCo;
import de.se_rwth.commons.logging.Log;

public class PrimitiveTypesAreForbidden implements TypesASTPrimitiveTypeCoCo {
    @Override
    public void check(ASTPrimitiveType node) {
        Log.error(
                "Primitive types are forbidden",
                node.get_SourcePositionStart()
        );
    }
}
