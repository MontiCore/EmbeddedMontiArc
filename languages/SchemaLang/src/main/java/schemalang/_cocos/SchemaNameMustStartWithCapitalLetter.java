/* (c) https://github.com/MontiCore/monticore */
package schemalang._cocos;

import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTSchemaDefinition;

public class SchemaNameMustStartWithCapitalLetter implements SchemaLangASTSchemaDefinitionCoCo {

    public static final String ERROR_CODE = "0xA2001";
    public static final String ERROR_MSG_FORMAT = " Provided schema name '%s' is not valid. A schema name may only start with a capital letter.";

    @Override
    public void check(ASTSchemaDefinition schema) {
        String schemaName = schema.getName();
        boolean startsWithUpperCase = Character.isUpperCase(schemaName.charAt(0));

        if (!startsWithUpperCase) {
            Log.error(ERROR_CODE.concat(String.format(ERROR_MSG_FORMAT, schemaName)),
                    schema.get_SourcePositionStart());
        }
    }
}