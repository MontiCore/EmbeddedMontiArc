package schemalang._cocos;

import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTTypedDeclaration;
import schematypes.TypeIdentifier;
import schematypes._ast.ASTObjectType;
import schematypes._ast.ASTSchemaType;

import static schemalang.ErrorCodes.*;

public class CustomPropertyHasDifferentName implements SchemaLangASTTypedDeclarationCoCo {

    @Override
    public void check(ASTTypedDeclaration attribute) {
        ASTSchemaType type = attribute.getType();
        if (!TypeIdentifier.OBJECT.equals(type.getSchemaLangType())) {
            return;
        }

        ASTObjectType complexType = (ASTObjectType) attribute.getType();
        String genericType = complexType.getGenericType();

        boolean sameName = attribute.getName().equals(genericType);
        if (sameName) {
            Log.error(ERROR_CODE_SL_09C.concat(String.format(ERROR_MSG_SL_09C, genericType)),
                    attribute.get_SourcePositionStart());
        }
    }
}