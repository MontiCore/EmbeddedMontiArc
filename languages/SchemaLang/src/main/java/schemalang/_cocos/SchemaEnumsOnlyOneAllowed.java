package schemalang._cocos;

import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTNestedSchemaEnumLinkDefinition;
import schemalang._ast.ASTSchemaDefinition;

import java.util.List;

import static schemalang.ErrorCodes.*;

public class SchemaEnumsOnlyOneAllowed implements SchemaLangASTSchemaDefinitionCoCo {

    @Override
    public void check(ASTSchemaDefinition node) {

        List<ASTNestedSchemaEnumLinkDefinition> schemaEnumLinkDefinitions =
                node.getSchemaEnumLinkDefinitions();

        if (schemaEnumLinkDefinitions != null && schemaEnumLinkDefinitions.size() > 1) {
            Log.error(ERROR_CODE_SL_22C.concat(ERROR_MSG_SL_22C),
                    node.get_SourcePositionStart());
        }
    }
}