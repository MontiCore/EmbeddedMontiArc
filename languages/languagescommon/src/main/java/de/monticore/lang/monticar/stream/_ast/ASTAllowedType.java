package de.monticore.lang.monticar.stream._ast;

import de.monticore.literals.literals._ast.ASTSignedLiteral;

import java.util.Optional;

public class ASTAllowedType extends ASTAllowedTypeTOP {

    protected ASTAllowedType() {}

    protected ASTAllowedType(Optional<ASTSignedLiteral> num, Optional<ASTFilePath> filePath) {
        super(num, filePath);
    }
}
