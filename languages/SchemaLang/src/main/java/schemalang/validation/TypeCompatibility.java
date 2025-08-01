package schemalang.validation;

import conflangliterals._ast.ASTListLiteral;
import de.monticore.mcliterals._ast.ASTSignedDoubleLiteral;
import de.monticore.mcliterals._ast.ASTSignedIntLiteral;
import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.se_rwth.commons.logging.Log;
import schemalang.validation.exception.TypeNotSupportedException;
import schematypes._ast.*;

import java.util.List;

import static schemalang.validation.ValidationHelpers.*;

public class TypeCompatibility {

    public static boolean isValueCompatibleWithType(ASTSignedLiteral signedLiteral, ASTSchemaType schemaType) {

        if (schemaType.isEMAType()) {
            ASTEMAType emaType = (ASTEMAType) schemaType;
            return isValueCompatibleWithEMAType(signedLiteral, emaType);

        } else if (schemaType instanceof ASTListType) {
            ASTListType listType = (ASTListType) schemaType;
            return isValueCompatibleWithVectorType(signedLiteral, listType);

        } else if (schemaType instanceof ASTStringType) {
            return isString(signedLiteral) || isNull(signedLiteral);

        } else if (schemaType instanceof ASTComponentType) {
            return isComponent(signedLiteral) || isConstant(signedLiteral);

        } else if (schemaType instanceof ASTObjectType) {
            return isConstant(signedLiteral);
        }

        throw new TypeNotSupportedException("The specified type is not supported.", schemaType, schemaType.get_SourcePositionStart()); // TODO catch this exception
    }

    private static boolean isValueCompatibleWithEMAType(ASTSignedLiteral signedLiteral, ASTEMAType emaType) {

        if (emaType instanceof ASTNaturalNumberWithoutZeroType) {
            return isNaturalNumberWithoutZero(signedLiteral);

        } else if (emaType instanceof ASTNaturalNumberWithZeroType) {
            return isNaturalNumberWithZero(signedLiteral);

        } else if (emaType instanceof ASTWholeNumberType) {
            return isWholeNumber(signedLiteral);

        } else if (emaType instanceof ASTRationalNumberType) {
            return isRationalNumber(signedLiteral);

        } else if (emaType instanceof ASTStringType) {
            return isString(signedLiteral) || isNull(signedLiteral);

        } else if (emaType instanceof ASTBooleanType) {
            return isBoolean(signedLiteral);
        }

        throw new TypeNotSupportedException("The specified type is not supported.", emaType, emaType.get_SourcePositionStart());
    }

    private static boolean isValueCompatibleWithVectorType(ASTSignedLiteral signedLiteral, ASTListType listType) {
        if (!isList(signedLiteral)) {
            return false;
        }
        ASTListLiteral vectorLiteral = (ASTListLiteral) signedLiteral;
        List<ASTSignedLiteral> vectorValues = vectorLiteral.getSignedLiteralList();

        boolean isCompatible = true;
        for (ASTSignedLiteral literal : vectorValues) {
            isCompatible = isCompatible && isValueCompatibleWithType(literal, listType.getSchemaType());
        }
        return isCompatible;
    }

    public static Object getValueOfLiteral(ASTSignedLiteral numericLiteral) {

        if (numericLiteral instanceof ASTSignedIntLiteral) {
            ASTSignedIntLiteral signedIntLiteral = (ASTSignedIntLiteral) numericLiteral;
            return signedIntLiteral.getValue();

        } else if (numericLiteral instanceof ASTSignedDoubleLiteral) {
            ASTSignedDoubleLiteral signedDoubleLiteral = (ASTSignedDoubleLiteral) numericLiteral;
            return signedDoubleLiteral.getValue();
        }

        Log.warn(String.format("Type '%s' is not supported.", numericLiteral));
        return null;
    }

    /*
    ASTMatrixType (schematypes._ast)
    ASTEMAType (schematypes._ast)
    ASTRationalNumberType (schematypes._ast)
    ASTIntegerNumberType (schematypes._ast)
    ASTListType (schematypes._ast)
    ASTStringType (schematypes._ast)
    ASTNaturalNumberWithZeroType (schematypes._ast)
    ASTBooleanType (schematypes._ast)
    ASTNaturalNumberWithoutZeroType (schematypes._ast)
    ASTClassSchemaType (schematypes._ast)
    ASTElementType (schematypes._ast)
    ASTComponentType (schematypes._ast)
    */

}