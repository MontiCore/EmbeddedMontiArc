package schemalang._ast;

import de.monticore.mcliterals._ast.ASTSignedLiteral;
import schemalang.SchemaMemberType;
import schemalang._symboltable.NestedSchemaEnumLinkDefinitionSymbol;

import java.util.List;
import java.util.Optional;

import static schemalang.SchemaMemberType.SCHEMA;

public class ASTNestedSchemaEnumLinkDefinition extends ASTNestedSchemaEnumLinkDefinitionTOP {

    public ASTNestedSchemaEnumLinkDefinition() {
    }

    public ASTNestedSchemaEnumLinkDefinition(Optional<ASTSignedLiteral> initial, List<ASTSchemaLink> linkss, String name) {
        super(initial, linkss, name);
    }

    @Override
    public SchemaMemberType getSchemaMemberType() {
        return SCHEMA;
    }

    @Override
    public Optional<NestedSchemaEnumLinkDefinitionSymbol> getNestedSchemaEnumLinkDefinitionSymbolOpt() {
        if (symbol.isPresent()) {
            NestedSchemaEnumLinkDefinitionSymbol linkDefinitionSymbol = (NestedSchemaEnumLinkDefinitionSymbol) symbol.get();
            return Optional.of(linkDefinitionSymbol);
        }
        return super.getNestedSchemaEnumLinkDefinitionSymbolOpt();
    }
}