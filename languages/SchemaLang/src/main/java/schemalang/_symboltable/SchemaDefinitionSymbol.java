package schemalang._symboltable;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import schemalang.exception.SchemaLangTechnicalException;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.Set;

public class SchemaDefinitionSymbol extends schemalang._symboltable.SchemaDefinitionSymbolTOP {

    private Set<SchemaDefinitionSymbol> superSchemas = Sets.newHashSet();
    private Set<SchemaDefinitionSymbol> importedSchemas = Sets.newHashSet();
    private Set<NestedSchemaEnumLinkDefinitionSymbol> schemaEnumLinkDefinitionSymbols = Sets.newHashSet();
    private Collection<ReferenceModel> referenceModels = Lists.newArrayList();

    public SchemaDefinitionSymbol(String name) {
        super(name);
    }

    public boolean isInHierarchy(SchemaDefinitionSymbol symbol) {
        if (this.equals(symbol)) {
            return true;
        }

        if (superSchemas != null && !superSchemas.isEmpty()) {
            for (SchemaDefinitionSymbol superSchema : superSchemas) {
                if (superSchema.isInHierarchy(symbol)) return true;
            }
        }
        return false;
    }

    public boolean hasDeclaration(String name) {
        return hasAttributeEntry(name, TypedDeclarationSymbol.KIND)
                || hasAttributeEntry(name, EnumeratedDeclarationSymbol.KIND)
                || hasAttributeEntry(name, NestedSchemaEnumLinkDefinitionSymbol.KIND);
    }

    public Optional<TypedDeclarationSymbol> getTypedDeclaration(String name) {
        return getSchemaMemberOfKind(name, TypedDeclarationSymbol.KIND, true);
    }

    public Optional<EnumeratedDeclarationSymbol> getEnumeratedDeclaration(String name) {
        return getSchemaMemberOfKind(name, EnumeratedDeclarationSymbol.KIND, true);
    }

    public Optional<NestedSchemaEnumLinkDefinitionSymbol> getAttributeEntryOfKindEnumSchemaLinks(String name) {
        return getSchemaMemberOfKind(name, NestedSchemaEnumLinkDefinitionSymbol.KIND, true);
    }

    public Optional<ComplexPropertyDefinitionSymbol> getAttributeEntryOfKindComplex(String name) {
        Optional<ComplexPropertyDefinitionSymbol> complexAttributeDefinitionOpt = getSchemaMemberOfKind(name, ComplexPropertyDefinitionSymbol.KIND, true);
        if (complexAttributeDefinitionOpt.isPresent()) {
            return complexAttributeDefinitionOpt;
        }
        return getImportedComplexDefinition(name);
    }

    private <T extends Symbol> Optional<T> getSchemaMemberOfKind(String name, SymbolKind kind, boolean includeSuperSchemas) {
        Scope spannedScope = getSpannedScope();
        Optional<T> schemaMemberOpt = spannedScope.resolve(name, kind);
        if (schemaMemberOpt.isPresent()) {
            return schemaMemberOpt;
        }

        if (superSchemas != null && !superSchemas.isEmpty() && includeSuperSchemas) {
            for (SchemaDefinitionSymbol superSchema : superSchemas) {
                spannedScope = superSchema.getSpannedScope();
                schemaMemberOpt = spannedScope.resolve(name, kind);
                if (!schemaMemberOpt.isPresent()) {
                    schemaMemberOpt = superSchema.getSchemaMemberOfKind(name, kind, true);
                }

                if (schemaMemberOpt.isPresent()) {
                    return schemaMemberOpt;
                }
            }
        }
        return Optional.empty();
    }

    private boolean hasAttributeEntry(final String name, SymbolKind kind) {
        Scope spannedScope;
        Optional<SchemaMemberSymbol> schemaMemberOpt;
        if (superSchemas != null && !superSchemas.isEmpty()) {
            for (SchemaDefinitionSymbol superSchema : superSchemas) {
                spannedScope = superSchema.getSpannedScope();
                schemaMemberOpt = spannedScope.resolve(name, kind);
                if (schemaMemberOpt.isPresent()) return true;
            }
        }

        spannedScope = getSpannedScope();
        schemaMemberOpt = spannedScope.resolve(name, kind);
        return schemaMemberOpt.isPresent();
    }

    private Optional<ComplexPropertyDefinitionSymbol> getImportedComplexDefinition(String name) {
        Scope spannedScope;
        Optional<ComplexPropertyDefinitionSymbol> attributeDefinitionSymbolOpt = Optional.empty();
        if (importedSchemas != null && !importedSchemas.isEmpty()) {
            for (SchemaDefinitionSymbol superSchema : importedSchemas) {
                spannedScope = superSchema.getSpannedScope();
                attributeDefinitionSymbolOpt = spannedScope.resolve(name, ComplexPropertyDefinitionSymbol.KIND);
                if (attributeDefinitionSymbolOpt.isPresent()) return attributeDefinitionSymbolOpt;
            }
        }

        if (superSchemas != null && !superSchemas.isEmpty()) {
            for (SchemaDefinitionSymbol superSchema : superSchemas) {
                attributeDefinitionSymbolOpt = superSchema.getImportedComplexDefinition(name);
                if (attributeDefinitionSymbolOpt.isPresent()) return attributeDefinitionSymbolOpt;
            }
        }
        return attributeDefinitionSymbolOpt;
    }

    public boolean hasDefault(String name) {
        Optional<Symbol> schemaMemberSymbolOpt = getSchemaMemberOfKind(name, TypedDeclarationSymbol.KIND, true);
        if (schemaMemberSymbolOpt.isPresent()) {
            TypedDeclarationSymbol symbol = (TypedDeclarationSymbol) schemaMemberSymbolOpt.get();
            return symbol.getDefaultValue().isPresent();
        }

        schemaMemberSymbolOpt = getSchemaMemberOfKind(name, EnumeratedDeclarationSymbol.KIND, true);
        if (schemaMemberSymbolOpt.isPresent()) {
            EnumeratedDeclarationSymbol symbol = (EnumeratedDeclarationSymbol) schemaMemberSymbolOpt.get();
            return symbol.hasDefaultValue();
        }

        schemaMemberSymbolOpt = getSchemaMemberOfKind(name, NestedSchemaEnumLinkDefinitionSymbol.KIND, true);
        if (schemaMemberSymbolOpt.isPresent()) {
            NestedSchemaEnumLinkDefinitionSymbol symbol = (NestedSchemaEnumLinkDefinitionSymbol) schemaMemberSymbolOpt.get();
            return symbol.hasDefaultValue();
        }

        throw new SchemaLangTechnicalException("Schema member '" +  name + "' does not exist.");
    }

    public <T> Optional<T> getDefaultValue(String name) {
        Optional<Symbol> schemaMemberSymbolOpt = getSchemaMemberOfKind(name, TypedDeclarationSymbol.KIND, true);
        if (schemaMemberSymbolOpt.isPresent()) {
            TypedDeclarationSymbol symbol = (TypedDeclarationSymbol) schemaMemberSymbolOpt.get();
            if (symbol.getDefaultValue().isPresent()) {
                return symbol.getDefaultValue().map(configurationEntry -> (T) configurationEntry);
            }
            return Optional.empty();
        }

        schemaMemberSymbolOpt = getSchemaMemberOfKind(name, EnumeratedDeclarationSymbol.KIND, true);
        if (schemaMemberSymbolOpt.isPresent()) {
            EnumeratedDeclarationSymbol symbol = (EnumeratedDeclarationSymbol) schemaMemberSymbolOpt.get();
            if (symbol.hasDefaultValue()) {
                return symbol.getDefaultValue().map(configurationEntry -> (T) configurationEntry);
            }
            return Optional.empty();
        }

        schemaMemberSymbolOpt = getSchemaMemberOfKind(name, NestedSchemaEnumLinkDefinitionSymbol.KIND, true);
        if (schemaMemberSymbolOpt.isPresent()) {
            NestedSchemaEnumLinkDefinitionSymbol symbol = (NestedSchemaEnumLinkDefinitionSymbol) schemaMemberSymbolOpt.get();
            if (symbol.hasDefaultValue()) {
                return symbol.getDefaultValue().map(configurationEntry -> (T) configurationEntry);
            }
            return Optional.empty();
        }

        throw new SchemaLangTechnicalException("Schema member '" +  name + "' does not exist.");
    }

    public void addSuperSchema(SchemaDefinitionSymbol symbol) {
        superSchemas.add(symbol);
    }

    public void addImportedSchema(SchemaDefinitionSymbol symbol) {
        importedSchemas.add(symbol);
    }

    public void addSchemaEnumLinkDefinitionSymbol(NestedSchemaEnumLinkDefinitionSymbol symbol) {
        schemaEnumLinkDefinitionSymbols.add(symbol);
    }

    public Set<NestedSchemaEnumLinkDefinitionSymbol> getSchemaEnumLinkDefinitionSymbols() {
        return schemaEnumLinkDefinitionSymbols;
    }

    public Collection<ReferenceModel> getReferenceModels() {
        ArrayList<ReferenceModel> referenceModels = Lists.newArrayList(this.referenceModels);
        if (superSchemas != null && !superSchemas.isEmpty()) {
            for (SchemaDefinitionSymbol superSchema : superSchemas) {
                referenceModels.addAll(superSchema.getReferenceModels());
            }
        }
        return referenceModels;
    }

    public void setReferenceModels(Collection<ReferenceModel> referenceModels) {
        this.referenceModels = referenceModels;
    }
}