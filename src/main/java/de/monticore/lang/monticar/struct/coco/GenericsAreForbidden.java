/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct.coco;

import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._cocos.TypesASTSimpleReferenceTypeCoCo;
import de.se_rwth.commons.logging.Log;

public class GenericsAreForbidden implements TypesASTSimpleReferenceTypeCoCo {
    @Override
    public void check(ASTSimpleReferenceType node) {
        if (node.getTypeArgumentsOpt().isPresent()) {
            Log.error(
                    "Generics are forbidden",
                    node.getTypeArgumentsOpt().get().get_SourcePositionStart()
            );
        }
    }
}
