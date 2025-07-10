package schemalang.validation;

import com.google.common.collect.Lists;
import conflangliterals._ast.ASTComponentLiteral;
import conflangliterals._ast.ASTListLiteral;
import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.mcliterals._ast.*;
import schemalang._symboltable.ReferenceModel;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang.validation.exception.TypeNotSupportedException;
import schematypes._ast.ASTDomain;
import schematypes._ast.ASTNumberWithInf;
import schematypes._ast.ASTRange;

import java.math.BigDecimal;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class ValidationHelpers {

    private ValidationHelpers() {
        // hidden constructor
    }

    public static boolean isBoolean(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTBooleanLiteral;
    }

    public static boolean isNaturalNumberWithoutZero(ASTSignedLiteral signedLiteral) {
        if (!isWholeNumber(signedLiteral)) {
            return false;
        }

        // TODO: Add support for long

        ASTSignedIntLiteral signedIntLiteral = (ASTSignedIntLiteral) signedLiteral;
        return isPositive(signedIntLiteral) && !isZero(signedIntLiteral);
    }

    public static boolean isNaturalNumberWithZero(ASTSignedLiteral signedLiteral) {
        if (!isWholeNumber(signedLiteral)) {
            return false;
        }

        // TODO: Add support for long

        ASTSignedIntLiteral signedIntLiteral = (ASTSignedIntLiteral) signedLiteral;
        return isPositive(signedIntLiteral);
    }

    public static boolean isRationalNumber(ASTSignedLiteral signedLiteral) {
        return isWholeNumber(signedLiteral) || isFloat(signedLiteral) || isDouble(signedLiteral);
    }

    public static boolean isWholeNumber(ASTSignedLiteral signedLiteral) {
        return isInteger(signedLiteral) || isLong(signedLiteral);
    }

    public static boolean isInteger(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTSignedIntLiteral;
    }

    public static boolean isLong(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTSignedLongLiteral;
    }

    public static boolean isFloat(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTSignedFloatLiteral;
    }

    public static boolean isDouble(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTSignedDoubleLiteral;
    }

    public static boolean isNull(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTNullLiteral;
    }

    public static boolean isZero(ASTSignedIntLiteral signedIntLiteral) {
        return signedIntLiteral.getValue() == 0;
    }

    public static boolean isPositive(ASTSignedIntLiteral signedIntLiteral) {
        return !signedIntLiteral.isNegative();
    }

    public static boolean isString(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTStringLiteral;
    }

    public static boolean isCharacter(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTCharLiteral;
    }

    public static boolean isList(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTListLiteral;
    }

    public static boolean isComponent(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTComponentLiteral;
    }

    public static boolean isConstant(ASTSignedLiteral signedLiteral) {
        return signedLiteral instanceof ASTTypelessLiteral;
    }

    public static boolean isInDomain(ASTSignedLiteral signedLiteral, ASTDomain values) {

        List<ASTSignedLiteral> domainValues = values.getValuesList();
        for (ASTSignedLiteral domainValue : domainValues) {
            if (domainValue.deepEquals(signedLiteral)) {
                return true;
            }
        }
        return false;
    }

    public static boolean isInRange(ASTSignedLiteral signedLiteral, ASTRange range) {

        if (!(signedLiteral instanceof ASTSignedNumericLiteral)) {
            return false;
        }

        ASTSignedNumericLiteral signedNumericLiteral = (ASTSignedNumericLiteral) signedLiteral;
        BigDecimal value = toBigDecimal(signedNumericLiteral);
        return range.isInRange(value);
    }

    public static boolean isOfScale(ASTSignedLiteral signedLiteral, ASTNumberWithInf stepResolution) {

        if (!(signedLiteral instanceof ASTSignedNumericLiteral)) {
            return false;
        }

        ASTSignedNumericLiteral signedNumericLiteral = (ASTSignedNumericLiteral) signedLiteral;
        return isOfScaleInternal(signedNumericLiteral, stepResolution);
    }

    private static boolean isOfScaleInternal(ASTSignedNumericLiteral signedNumericLiteral, ASTNumberWithInf stepResolution) {

        ASTSignedNumericLiteral scaleNumber = stepResolution.getNumber();

        BigDecimal literalValue = toBigDecimal(signedNumericLiteral);
        BigDecimal scaleValue = toBigDecimal(scaleNumber);
        BigDecimal remainder = literalValue.remainder(scaleValue);

        return remainder.compareTo(BigDecimal.ZERO) == 0;
    }

    public static BigDecimal toBigDecimal(ASTSignedNumericLiteral signedNumericLiteral) {

        if (isDouble(signedNumericLiteral)) {
            ASTSignedDoubleLiteral signedDoubleLiteral = (ASTSignedDoubleLiteral) signedNumericLiteral;
            return BigDecimal.valueOf(signedDoubleLiteral.getValue());

        } else if (isInteger(signedNumericLiteral)) {
            ASTSignedIntLiteral signedIntLiteral = (ASTSignedIntLiteral) signedNumericLiteral;
            return BigDecimal.valueOf(signedIntLiteral.getValue());
        }

        throw new TypeNotSupportedException("The specified type is not supported.", null, signedNumericLiteral.get_SourcePositionStart());
    }

    public static Collection<EMAComponentSymbol> getReferenceModels(List<SchemaDefinitionSymbol> schemas) {
        Collection<EMAComponentSymbol> referenceModels = Lists.newArrayList();
        for (SchemaDefinitionSymbol schema : schemas) {
            if (schema.getReferenceModels() == null) {
                continue;
            }
            referenceModels.addAll(schema.getReferenceModels().stream()
                    .map(ReferenceModel::getEMAComponent).collect(Collectors.toList()));
        }
        return referenceModels;
    }
}