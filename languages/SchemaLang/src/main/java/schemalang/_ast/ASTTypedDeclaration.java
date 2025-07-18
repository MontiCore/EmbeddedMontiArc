package schemalang._ast;

import de.monticore.mcliterals._ast.ASTSignedLiteral;
import schemalang.SchemaMemberType;
import schemalang._symboltable.TypedDeclarationSymbol;
import schematypes._ast.ASTSchemaType;

import java.util.Optional;

public class ASTTypedDeclaration extends ASTTypedDeclarationTOP {

    public ASTTypedDeclaration() {
        super();
    }

    public ASTTypedDeclaration(Optional<ASTSignedLiteral> initial, ASTSchemaType type, boolean required, String name) {
        super(initial, type, required, name);
    }

    @Override
    public SchemaMemberType getSchemaMemberType() {
        return SchemaMemberType.BASIC;
    }

    @Override
    public Optional<TypedDeclarationSymbol> getTypedDeclarationSymbolOpt() {
        if (symbol.isPresent()) {
            TypedDeclarationSymbol schemaSimpleAttributeSymbol = (TypedDeclarationSymbol) symbol.get();
            return Optional.of(schemaSimpleAttributeSymbol);
        }
        return super.getTypedDeclarationSymbolOpt();
    }
}