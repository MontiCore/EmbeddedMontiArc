package schemalang._symboltable;

import java.util.Collection;
import java.util.Optional;

public class EnumeratedDeclarationSymbol extends EnumeratedDeclarationSymbolTOP {

    private Object defaultValue;
    private Collection<String> enumerationValues;

    public EnumeratedDeclarationSymbol(String name) {
        super(name);
    }

    public Optional<Object> getDefaultValue() {
        return Optional.ofNullable(defaultValue);
    }

    public void setDefaultValue(Object defaultValue) {
        this.defaultValue = defaultValue;
    }

    public boolean hasDefaultValue() {
        return getDefaultValue().isPresent();
    }

    public Collection<String> getEnumerations() {
        return enumerationValues;
    }

    public boolean isValidEnumeration(String name) {
        return enumerationValues != null && enumerationValues.contains(name);
    }

    public void setEnumerationValues(Collection<String> enumerationValues) {
        this.enumerationValues = enumerationValues;
    }
}