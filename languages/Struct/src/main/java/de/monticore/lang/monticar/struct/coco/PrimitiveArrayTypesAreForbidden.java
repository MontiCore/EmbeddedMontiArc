/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct.coco;

import de.monticore.types.types._ast.ASTPrimitiveArrayType;
import de.monticore.types.types._cocos.TypesASTPrimitiveArrayTypeCoCo;
import de.se_rwth.commons.logging.Log;

public class PrimitiveArrayTypesAreForbidden implements TypesASTPrimitiveArrayTypeCoCo {
    @Override
    public void check(ASTPrimitiveArrayType node) {
        Log.error(
                "Primitive array types are forbidden",
                node.get_SourcePositionStart()
        );
    }
}
