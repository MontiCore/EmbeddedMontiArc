package schematypes._ast;

import java.util.Optional;

public interface ASTTypeWithRange extends ASTTypeWithRangeTOP {

    Optional<ASTRange> getRangeOpt();
}
