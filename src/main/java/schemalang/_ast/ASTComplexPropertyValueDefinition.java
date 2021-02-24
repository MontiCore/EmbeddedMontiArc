package schemalang._ast;

import schemalang._symboltable.ComplexPropertyValueDefinitionSymbol;

import java.util.List;
import java.util.Optional;

public class ASTComplexPropertyValueDefinition extends ASTComplexPropertyValueDefinitionTOP {

    public ASTComplexPropertyValueDefinition() {
    }

    public ASTComplexPropertyValueDefinition(String name, List<ASTSchemaMember> schemaMembers) {
        super(name, schemaMembers);
    }

    @Override
    public Optional<ComplexPropertyValueDefinitionSymbol> getComplexPropertyValueDefinitionSymbolOpt() {
        if (symbol.isPresent()) {
            ComplexPropertyValueDefinitionSymbol complexPropertyValueDefinitionSymbol = (ComplexPropertyValueDefinitionSymbol) symbol.get();
            return Optional.of(complexPropertyValueDefinitionSymbol);
        }
        return super.getComplexPropertyValueDefinitionSymbolOpt();
    }
}