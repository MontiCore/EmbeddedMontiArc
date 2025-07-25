package schemalang._symboltable;

import com.google.common.collect.Lists;
import de.monticore.symboltable.Scope;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

public class ComplexPropertyDefinitionSymbol extends ComplexPropertyDefinitionSymbolTOP {

    private List<ComplexPropertyValueDefinitionSymbol> propertyValueDefinitions = Lists.newArrayList();
    private Optional<ComplexPropertyDefinitionSymbol> parentComplexPropertyOpt = Optional.empty();
    private final Collection<String> validValues;
    private final boolean isOverride;

    public ComplexPropertyDefinitionSymbol(String name) {
        super(name);
        this.validValues = null;
        this.isOverride = false;
    }

    public ComplexPropertyDefinitionSymbol(String name, Collection<String> validValues, boolean isOverride) {
        super(name);
        this.validValues = validValues;
        this.isOverride = isOverride;
    }

    public Class getNativeType() {
        return Boolean.class;
    }

    public Optional<TypedDeclarationSymbol> getSchemaDefinition(String name, String value) {
        Scope spannedScope = getSpannedScope();
        Optional<TypedDeclarationSymbol> schemaAttributeOpt = spannedScope.resolve(name, TypedDeclarationSymbol.KIND);

        if (schemaAttributeOpt.isPresent()) {
            return schemaAttributeOpt;
        }
        schemaAttributeOpt = getFromComplexPropertyDefinition(name, value);
        if (!schemaAttributeOpt.isPresent() && parentComplexPropertyOpt.isPresent() && !isOverride) {
            ComplexPropertyDefinitionSymbol parentComplexProperty = parentComplexPropertyOpt.get();
            schemaAttributeOpt = parentComplexProperty.getSchemaDefinition(name, value);
        }
        return schemaAttributeOpt;
    }

    private Optional<TypedDeclarationSymbol> getFromComplexPropertyDefinition(String name, String value) {
        if (propertyValueDefinitions.isEmpty()) {
            return Optional.empty();
        }

        for (ComplexPropertyValueDefinitionSymbol propertyValueDefinition : propertyValueDefinitions) {
            if (!propertyValueDefinition.getName().equals(value)) {
                continue;
            }
            return propertyValueDefinition.getSchemaDefinition(name);
        }
        return Optional.empty();
    }

    public boolean isValidValue(String instance) {
        boolean isValid = isValidValueInternal(instance);
        if (!isValid && parentComplexPropertyOpt.isPresent() && !isOverride) {
            ComplexPropertyDefinitionSymbol parentComplexProperty = parentComplexPropertyOpt.get();
            return parentComplexProperty.isValidValue(instance);
        }
        return isValid;
    }

    public Collection<String> getValidValues() {
        return validValues;
    }

    public void addPropertyValueDefinition(ComplexPropertyValueDefinitionSymbol symbol) {
        propertyValueDefinitions.add(symbol);
    }

    public void setParentComplexProperty(ComplexPropertyDefinitionSymbol parentComplexProperty) {
        this.parentComplexPropertyOpt = Optional.ofNullable(parentComplexProperty);
    }

    private boolean isValidValueInternal(Object instance) {
        if (validValues == null || validValues.isEmpty()) {
            return false;
        }
        return validValues.contains(instance);
    }
}