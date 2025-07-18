package schemalang._cocos;

import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTTypedDeclaration;
import schemalang.validation.TypeCompatibility;
import schematypes._ast.ASTDomain;
import schematypes._ast.ASTSchemaType;
import schematypes._ast.ASTTypeWithDomain;

import java.util.List;
import java.util.Optional;

import static schemalang.ErrorCodes.ERROR_CODE_SL_06C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_06C;
import static schemalang.validation.TypeCompatibility.getValueOfLiteral;

public class DomainValuesCompatibleWithType implements SchemaLangASTTypedDeclarationCoCo {

    @Override
    public void check(ASTTypedDeclaration attribute) {

        ASTSchemaType type = attribute.getType();
        if (!type.isTypeWithDomain()) {
            return;
        }

        ASTTypeWithDomain typeWithDomain = (ASTTypeWithDomain) type;
        Optional<ASTDomain> domainOpt = typeWithDomain.getValuesOpt();
        if (!domainOpt.isPresent()) {
            return;
        }

        List<ASTSignedLiteral> domainValues = domainOpt.get().getValuesList();
        for (ASTSignedLiteral domainValue : domainValues) {
            if (!TypeCompatibility.isValueCompatibleWithType(domainValue, type)) {
                Log.error(ERROR_CODE_SL_06C.concat(String.format(ERROR_MSG_SL_06C, getValueOfLiteral(domainValue), attribute.getName(), type)),
                        attribute.get_SourcePositionStart());
            }
        }
    }
}