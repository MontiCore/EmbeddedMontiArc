package schemalang._symboltable;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import java.util.List;
import java.util.Map;
import java.util.Optional;

public class NestedSchemaEnumLinkDefinitionSymbol extends NestedSchemaEnumLinkDefinitionSymbolTOP {

    private Object defaultValue;
    private Map<String, String> schemaLinkMap = Maps.newHashMap();

    public NestedSchemaEnumLinkDefinitionSymbol(String name) {
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

    public void addSchemaLink(String key, String schema) {
        schemaLinkMap.put(key, schema);
    }

    public String getSchemaLink(String key) {
        return schemaLinkMap.get(key);
    }

    public List<NestedSchemaEnumLinkDefinitionSymbol> getSchemaLinkDefinitionSymbols() {
        return Lists.newArrayList();
    }
}