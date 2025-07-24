package schematypes._ast;

import de.monticore.mcliterals._ast.ASTSignedNumericLiteral;
import schemalang.validation.TypeCompatibility;

import java.math.BigDecimal;
import java.util.Optional;

import static schemalang.validation.ValidationHelpers.toBigDecimal;

public class ASTLeftOpenRange extends ASTLeftOpenRangeTOP {

    public ASTLeftOpenRange() {
    }

    public ASTLeftOpenRange(ASTNumberWithInf min, Optional<ASTNumberWithInf> step, ASTNumberWithInf max, String left, String right) {
        super(min, step, max, left, right);
    }

    @Override
    public boolean isInRange(BigDecimal value) {
        ASTNumberWithInf min = getMin();
        ASTSignedNumericLiteral minRangeNumericLiteral = min.getNumber();

        ASTNumberWithInf max = getMax();
        ASTSignedNumericLiteral maxRangeNumericLiteral = max.getNumber();

        BigDecimal minRangeValue = toBigDecimal(minRangeNumericLiteral);
        BigDecimal maxRangeValue = toBigDecimal(maxRangeNumericLiteral);
        return (value.compareTo(minRangeValue) > 0 && value.compareTo(maxRangeValue) <= 0);
    }

    @Override
    public String toString() {
        ASTNumberWithInf min = getMin();
        ASTNumberWithInf max = getMax();

        ASTSignedNumericLiteral minLiteral = min.getNumber();
        ASTSignedNumericLiteral maxLiteral = max.getNumber();

        String builder = "(" +
                TypeCompatibility.getValueOfLiteral(minLiteral) +
                ":" +
                TypeCompatibility.getValueOfLiteral(maxLiteral) +
                "]";
        return builder;
    }
}