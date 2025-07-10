package schemalang._cocos;

import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTTypedDeclaration;
import schematypes._ast.ASTDomain;
import schematypes._ast.ASTSchemaType;
import schematypes._ast.ASTTypeWithDomain;

import java.util.Optional;

import static conflangliterals.LiteralHelpers.literalToString;
import static schemalang.ErrorCodes.ERROR_CODE_SL_05C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_05C;
import static schemalang.validation.ValidationHelpers.isInDomain;

public class DefaultValueIsInDomain implements SchemaLangASTTypedDeclarationCoCo {

    @Override
    public void check(ASTTypedDeclaration attribute) {

        ASTSchemaType type = attribute.getType();
        if (!attribute.isPresentInitial() || !type.isTypeWithDomain()) {
            return;
        }

        ASTTypeWithDomain typeWithDomain = (ASTTypeWithDomain) type;
        ASTSignedLiteral initialValue = attribute.getInitial();
        Optional<ASTDomain> domainOpt = typeWithDomain.getValuesOpt();
        if (!domainOpt.isPresent()) {
            return;
        }

        boolean isInDomain = isInDomain(initialValue, domainOpt.get());
        if (!isInDomain) {
            Log.error(ERROR_CODE_SL_05C.concat(String.format(ERROR_MSG_SL_05C, literalToString(initialValue),
                    attribute.getName(), domainOpt.get())), attribute.get_SourcePositionStart());
        }
    }
}