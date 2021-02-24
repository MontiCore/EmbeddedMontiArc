package schemalang._ast;

import de.monticore.mcliterals._ast.ASTSignedLiteral;
import schemalang.SchemaMemberType;
import schemalang._symboltable.EnumeratedDeclarationSymbol;

import java.util.List;
import java.util.Optional;

public class ASTEnumeratedDeclaration extends ASTEnumeratedDeclarationTOP {

    public ASTEnumeratedDeclaration() {
    }

    public ASTEnumeratedDeclaration(Optional<ASTSignedLiteral> initial, List<ASTSchemaConstant> enumss, boolean required, String name) {
        super(initial, enumss, required, name);
    }

    @Override
    public SchemaMemberType getSchemaMemberType() {
        return SchemaMemberType.ENUM;
    }

    @Override
    public Optional<EnumeratedDeclarationSymbol> getEnumeratedDeclarationSymbolOpt() {
        if (symbol.isPresent()) {
            EnumeratedDeclarationSymbol schemaEnumAttributeSymbol = (EnumeratedDeclarationSymbol) symbol.get();
            return Optional.of(schemaEnumAttributeSymbol);
        }
        return super.getEnumeratedDeclarationSymbolOpt();
    }
}