package schemalang._cocos;

import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTTypedDeclaration;
import schematypes._ast.ASTRange;
import schematypes._ast.ASTSchemaType;
import schematypes._ast.ASTTypeWithRange;

import java.util.Optional;

import static conflangliterals.LiteralHelpers.literalToString;
import static schemalang.ErrorCodes.ERROR_CODE_SL_24C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_24C;
import static schemalang.validation.ValidationHelpers.isInRange;

public class DefaultValueIsInRange implements SchemaLangASTTypedDeclarationCoCo {

    @Override
    public void check(ASTTypedDeclaration attribute) {

        ASTSchemaType type = attribute.getType();
        if (!attribute.isPresentInitial() || !type.isTypeWithRange()) {
            return;
        }

        ASTTypeWithRange typeWithRange = (ASTTypeWithRange) type;
        ASTSignedLiteral initialValue = attribute.getInitial();
        Optional<ASTRange> rangeOpt = typeWithRange.getRangeOpt();
        if (!rangeOpt.isPresent()) {
            return;
        }

        ASTRange range = rangeOpt.get();
        if (!isInRange(initialValue, range)) {
            Log.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C, literalToString(initialValue),
                    attribute.getName(), range)), attribute.get_SourcePositionStart());
        }
    }
}