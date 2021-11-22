package schematypes._ast;

import java.math.BigDecimal;
import java.util.Optional;

public interface ASTRange extends ASTRangeTOP {

    boolean isInRange(BigDecimal value);

    Optional<ASTNumberWithInf> getStepOpt();
}