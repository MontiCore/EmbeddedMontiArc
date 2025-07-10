package schemalang._ast;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.ScopeSpanningSymbol;
import schemalang.SchemaMemberType;
import schemalang._symboltable.ComplexPropertyDefinitionSymbol;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang.exception.SchemaLangTechnicalException;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public class ASTComplexPropertyDefinition extends ASTComplexPropertyDefinitionTOP {

    private Optional<ASTComplexPropertyDefinition> superPropertyDefinition = Optional.empty();

    public ASTComplexPropertyDefinition() {
    }

    public ASTComplexPropertyDefinition(ASTComplexPropertyValues complexPropertyValues, List<ASTSchemaMember> schemaMembers,
                                        List<ASTComplexPropertyValueDefinition> complexPropertyValueDefinitions, boolean override, String name) {
        super(complexPropertyValues, schemaMembers, complexPropertyValueDefinitions, override, name);
    }

    @Override
    public SchemaMemberType getSchemaMemberType() {
        return SchemaMemberType.COMPLEX;
    }

    public List<String> getAllowedValues() {
        ASTComplexPropertyValues complexPropertyValues = getComplexPropertyValues();
        List<ASTSchemaConstant> valuesList = complexPropertyValues.getValuesList();
        if (valuesList.isEmpty()) {
            return Lists.newArrayList();
        }

        List<String> complexPropertyValuesAsString = Lists.newArrayList();
        for (ASTSchemaConstant schemaConstant : valuesList) {
            complexPropertyValuesAsString.add(schemaConstant.getName());
        }
        return complexPropertyValuesAsString;
    }

    public Map<String, List<String>> getAllowedPropertiesForValues() {

        Map<String, List<String>> allowedPropertiesForValues = Maps.newHashMap();
        List<ASTSchemaMember> generalProperties = getSchemaMemberList();
        if (generalProperties != null && generalProperties.isEmpty()) {
            List<String> allowedValues = getAllowedValues();
            for (String allowedValue : allowedValues) {
                allowedPropertiesForValues.put(allowedValue, generalProperties.stream().
                        map(ASTSchemaMemberTOP::getName).collect(Collectors.toList()));
            }
        }

        List<ASTComplexPropertyValueDefinition> valueDefinitions = getComplexPropertyValueDefinitionList();
        List<String> properties = Lists.newArrayList();
        for (ASTComplexPropertyValueDefinition valueDefinition : valueDefinitions) {
            if (valueDefinition.getSchemaMemberList() != null) {
                properties = valueDefinition.getSchemaMemberList().stream().
                        map(ASTSchemaMemberTOP::getName).collect(Collectors.toList());
            }

            if (allowedPropertiesForValues.containsKey(valueDefinition.getName())) {
                allowedPropertiesForValues.get(valueDefinition.getName()).addAll(properties);
                continue;
            }
            allowedPropertiesForValues.put(valueDefinition.getName(), Lists.newArrayList(properties));
        }
        return allowedPropertiesForValues;
    }

    @Override
    public Optional<ComplexPropertyDefinitionSymbol> getComplexPropertyDefinitionSymbolOpt() {
        if (symbol.isPresent()) {
            ComplexPropertyDefinitionSymbol complexAttributeDefinitionSymbol = (ComplexPropertyDefinitionSymbol) symbol.get();
            return Optional.of(complexAttributeDefinitionSymbol);
        }
        return super.getComplexPropertyDefinitionSymbolOpt();
    }

    public ASTSchemaDefinition getSchema() {
        Scope enclosingScope = getEnclosingScope(); // this should always be there
        Optional<? extends ScopeSpanningSymbol> spanningSymbol = enclosingScope.getSpanningSymbol();
        if (!spanningSymbol.isPresent()) {
            throw new SchemaLangTechnicalException("Symbol not available. Did you create the symbol table?");
        }
        SchemaDefinitionSymbol symbol = (SchemaDefinitionSymbol) spanningSymbol.get();
        Optional<ASTSchemaDefinition> schemaLangDefinitionNode = symbol.getSchemaDefinitionNode();
        return schemaLangDefinitionNode.get();
    }

    public Optional<ASTComplexPropertyDefinition> getSuperPropertyDefinition() {
        return superPropertyDefinition;
    }

    public void setParentComplexProperty(ASTComplexPropertyDefinition propertyDefinition) {
        this.superPropertyDefinition = Optional.of(propertyDefinition);
    }
}