package schemalang._symboltable;

import com.google.common.collect.Lists;

import java.util.List;
import java.util.Optional;

public class ComplexPropertyValueDefinitionSymbol extends ComplexPropertyValueDefinitionSymbolTOP {

    private List<TypedDeclarationSymbol> basicSchemaProperties = Lists.newArrayList();

    public ComplexPropertyValueDefinitionSymbol(String name) {
        super(name);
    }

    public void addBasicSchemaProperty(TypedDeclarationSymbol symbol) {
        basicSchemaProperties.add(symbol);
    }

    public Optional<TypedDeclarationSymbol> getSchemaDefinition(String name) {
        if (basicSchemaProperties.isEmpty()) {
            return Optional.empty();
        }

        for (TypedDeclarationSymbol basicSchemaProperty : basicSchemaProperties) {
            if (basicSchemaProperty.getName().equals(name)) {
                return Optional.of(basicSchemaProperty);
            }
        }
        return Optional.empty();
    }
}