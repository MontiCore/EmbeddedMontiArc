/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct.coco;

import de.monticore.types.types._ast.ASTComplexReferenceType;
import de.monticore.types.types._cocos.TypesASTComplexReferenceTypeCoCo;
import de.se_rwth.commons.logging.Log;

public class ComplexTypesAreForbidden implements TypesASTComplexReferenceTypeCoCo {
    @Override
    public void check(ASTComplexReferenceType node) {
        if (node.getSimpleReferenceTypeList().size() != 1) {
            Log.error(
                    "Complex types are forbidden",
                    node.get_SourcePositionStart()
            );
        }
    }
}
