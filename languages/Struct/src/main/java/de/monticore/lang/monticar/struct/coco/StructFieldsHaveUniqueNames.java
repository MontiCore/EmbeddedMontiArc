/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct.coco;

import de.monticore.lang.monticar.struct._ast.ASTStruct;
import de.monticore.lang.monticar.struct._ast.ASTStructFieldDefinition;
import de.monticore.lang.monticar.struct._cocos.StructASTStructCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class StructFieldsHaveUniqueNames implements StructASTStructCoCo {
    @Override
    public void check(ASTStruct node) {
        Set<String> fieldNames = new HashSet<>();
        List<ASTStructFieldDefinition> structFieldDefinitions = node.getStructFieldDefinitionList();
        for (ASTStructFieldDefinition field : structFieldDefinitions) {
            if (!fieldNames.add(field.getName())) {
                Log.error(
                        "Duplicate field",
                        field.get_SourcePositionStart()
                );
            }
        }
    }
}
