package schemalang.validation.exception;

import de.monticore.mcbasictypes1._ast.ASTType;
import de.se_rwth.commons.SourcePosition;

public class TypeNotSupportedException extends RuntimeException {

    public TypeNotSupportedException(String message, ASTType type, SourcePosition sourcePositionStart) {
        super(message);
    }
}