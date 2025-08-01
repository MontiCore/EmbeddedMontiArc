package schemalang._cocos;

import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTTypedDeclaration;
import schematypes._ast.ASTSchemaType;

import static conflangliterals.LiteralHelpers.literalToString;
import static schemalang.ErrorCodes.ERROR_CODE_SL_04C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_04C;
import static schemalang.validation.TypeCompatibility.isValueCompatibleWithType;

public class DefaultValueCompatibleWithType implements SchemaLangASTTypedDeclarationCoCo {

    @Override
    public void check(ASTTypedDeclaration attribute) {
        if (!attribute.isPresentInitial()) {
            return;
        }

        ASTSignedLiteral initialValue = attribute.getInitial();
        ASTSchemaType schemaType = attribute.getType();

        if (!isValueCompatibleWithType(initialValue, schemaType)) {
            Log.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C,
                    literalToString(initialValue), schemaType)),
                    attribute.get_SourcePositionStart());
        }
    }
}