
package schemalang._ast;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import schemalang.SchemaMemberType;
import schemalang._symboltable.SchemaDefinitionSymbol;

import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public class ASTSchemaDefinition extends ASTSchemaDefinitionTOP {

    private String fileName;

    private Set<ASTSchemaDefinition> superSchemas = Sets.newHashSet();

    public ASTSchemaDefinition() {
    }

    public ASTSchemaDefinition(String name, Optional<ASTSchemaExtendUsage> schemaExtendUsage, Optional<ASTReferenceModel> schemaReferenceModel, List<ASTSchemaMember> schemaMembers) {
        super(name, schemaExtendUsage, schemaReferenceModel, schemaMembers);
    }

    public String getFileName() {
        return fileName;
    }

    public void setFileName(String fileName) {
        this.fileName = fileName;
    }

    public void addSuperSchema(ASTSchemaDefinition ast) {
        superSchemas.add(ast);
    }

    public Set<ASTSchemaDefinition> getSuperSchemaDefinitions() {
        return superSchemas;
    }

    public Collection<ASTSchemaMember> getAllDeclarations() {
        Collection<ASTSchemaMember> allPropertyDefinitions = Lists.newArrayList();
        for (ASTSchemaMember schemaMember : getSchemaMemberList()) {
            if (!SchemaMemberType.REQUIRES_RULE.equals(schemaMember.getSchemaMemberType())) {
                allPropertyDefinitions.add(schemaMember);
            }
        }
        return allPropertyDefinitions;
    }

    public List<ASTTypedDeclaration> getBasicSchemaProperties() {
        List<ASTTypedDeclaration> basicSchemaProperties = Lists.newArrayList();
        for (ASTSchemaMember schemaMember : getSchemaMemberList()) {
            if (SchemaMemberType.BASIC.equals(schemaMember.getSchemaMemberType())) {
                basicSchemaProperties.add((ASTTypedDeclaration) schemaMember);
            }
        }
        return basicSchemaProperties;
    }

    public List<ASTEnumeratedDeclaration> getEnumPropertyDefinitions() {
        List<ASTEnumeratedDeclaration> enumSchemaProperties = Lists.newArrayList();
        for (ASTSchemaMember schemaMember : getSchemaMemberList()) {
            if (SchemaMemberType.ENUM.equals(schemaMember.getSchemaMemberType())) {
                enumSchemaProperties.add((ASTEnumeratedDeclaration) schemaMember);
            }
        }
        return enumSchemaProperties;
    }

    public List<ASTComplexPropertyDefinition> getComplexPropertyDefinitions() {
        List<ASTComplexPropertyDefinition> complexPropertyDefinitions = Lists.newArrayList();
        for (ASTSchemaMember schemaMember : getSchemaMemberList()) {
            if (SchemaMemberType.COMPLEX.equals(schemaMember.getSchemaMemberType())) {
                complexPropertyDefinitions.add((ASTComplexPropertyDefinition) schemaMember);
            }
        }
        return complexPropertyDefinitions;
    }

    public List<ASTRequiresRule> getRequiresRuleDefinitions() {
        List<ASTRequiresRule> requiresRules = Lists.newArrayList();
        for (ASTSchemaMember schemaMember : getSchemaMemberList()) {
            if (SchemaMemberType.REQUIRES_RULE.equals(schemaMember.getSchemaMemberType())) {
                requiresRules.add((ASTRequiresRule) schemaMember);
            }
        }
        return requiresRules;
    }

    public List<ASTNestedSchemaEnumLinkDefinition> getSchemaEnumLinkDefinitions() {
        List<ASTNestedSchemaEnumLinkDefinition> schemaEnumLinkDefinitions = Lists.newArrayList();
        for (ASTSchemaMember schemaMember : getSchemaMemberList()) {
            if (SchemaMemberType.SCHEMA.equals(schemaMember.getSchemaMemberType())) {
                schemaEnumLinkDefinitions.add((ASTNestedSchemaEnumLinkDefinition) schemaMember);
            }
        }
        return schemaEnumLinkDefinitions;
    }

    public Optional<ASTComplexPropertyDefinition> getComplexPropertyDefinition(String name) {
        List<ASTComplexPropertyDefinition> complexPropertyDefinitions = getComplexPropertyDefinitions();
        for (ASTComplexPropertyDefinition complexPropertyDefinition : complexPropertyDefinitions) {
            if (complexPropertyDefinition.getName().equals(name)) {
                return Optional.of(complexPropertyDefinition);
            }
        }
        return Optional.empty();
    }

    @Override
    public SchemaDefinitionSymbol getSchemaDefinitionSymbol() {
        return (SchemaDefinitionSymbol) symbol.orElse(null);
    }

    public Collection<ASTSchemaMember> getRequiredPropertyDefinitions() {
        Collection<ASTSchemaMember> requiredPropertyDefinitions = Lists.newArrayList();
        for (ASTTypedDeclaration basicSchemaProperty : getBasicSchemaProperties()) {
            if (basicSchemaProperty.isRequired()) {
                requiredPropertyDefinitions.add(basicSchemaProperty);
            }
        }
        for (ASTEnumeratedDeclaration enumSchemaProperty : getEnumPropertyDefinitions()) {
            if (enumSchemaProperty.isRequired()) {
                requiredPropertyDefinitions.add(enumSchemaProperty);
            }
        }
        return requiredPropertyDefinitions;
    }
}