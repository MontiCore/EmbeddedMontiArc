package schemalang._cocos;

import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTComplexPropertyDefinition;
import schemalang._ast.ASTSchemaDefinition;

import java.util.Optional;
import java.util.Set;

import static schemalang.ErrorCodes.*;

public class CustomPropertyDefinitionOverrideOnlyInSubSchema implements SchemaLangASTComplexPropertyDefinitionCoCo {

    @Override
    public void check(ASTComplexPropertyDefinition node) {

        if (!node.isOverride()) {
            return;
        }

        ASTSchemaDefinition schema = node.getSchema();
        Set<ASTSchemaDefinition> superSchemaDefinitions = schema.getSuperSchemaDefinitions();
        if (superSchemaDefinitions.isEmpty()) {
            Log.error(ERROR_CODE_SL_17C.concat(String.format(ERROR_MSG_SL_17C, node.getName(), schema.getName())),
                    node.get_SourcePositionStart());
            return;
        }

        Optional<ASTComplexPropertyDefinition> superPropertyDefinition = node.getSuperPropertyDefinition();
        if (!superPropertyDefinition.isPresent()) {
            Log.error(ERROR_CODE_SL_18C.concat(String.format(ERROR_MSG_SL_18C, node.getName())),
                    node.get_SourcePositionStart());
        }
    }
}