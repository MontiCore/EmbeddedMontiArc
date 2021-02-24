package schemalang._cocos;

import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTTypedDeclaration;
import schematypes._ast.*;

import java.util.Optional;

import static schemalang.ErrorCodes.ERROR_CODE_SL_23C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_23C;

public class RangeAndDomainDontCoOccur implements SchemaLangASTTypedDeclarationCoCo {

    @Override
    public void check(ASTTypedDeclaration attribute) {
        ASTSchemaType type = attribute.getType();

        // Only to be checked if the given type supports both a range and domain
        if (!type.isTypeWithRange() || !type.isTypeWithDomain()) {
            return;
        }

        ASTTypeWithDomain typeWithDomain = (ASTTypeWithDomain) type;
        Optional<ASTDomain> domainOpt = typeWithDomain.getValuesOpt();
        if (!domainOpt.isPresent()) {
            return;
        }

        ASTTypeWithRange typeWithRange = (ASTTypeWithRange) type;
        Optional<ASTRange> rangeOpt = typeWithRange.getRangeOpt();
        if (!rangeOpt.isPresent()) {
            return;
        }

        Log.error(ERROR_CODE_SL_23C.concat(String.format(ERROR_MSG_SL_23C, attribute.getName())),
                attribute.get_SourcePositionStart());
    }
}