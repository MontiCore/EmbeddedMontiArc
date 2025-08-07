package schemalang._cocos;

import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTSchemaDefinition;

import static schemalang.ErrorCodes.ERROR_CODE_SL_08C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_08C;

public class FileAndSchemaNameAreEqual implements SchemaLangASTSchemaDefinitionCoCo {

    @Override
    public void check(ASTSchemaDefinition ast) {

        String fileName = ast.getFileName();
        String schemaName = ast.getName();

        if (!fileName.equals(schemaName)) {
            Log.error(ERROR_CODE_SL_08C.concat(ERROR_MSG_SL_08C), ast.get_SourcePositionStart());
        }
    }
}