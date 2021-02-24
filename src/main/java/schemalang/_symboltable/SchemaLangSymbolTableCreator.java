package schemalang._symboltable;

import com.google.common.base.Joiner;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.mcbasictypes1._ast.ASTImportStatement;
import de.monticore.mcbasictypes1._ast.ASTQualifiedName;
import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.monticore.symboltable.*;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FilenameUtils;
import schemalang.SchemaMemberType;
import schemalang._ast.*;
import schemalang._cocos.SchemaLangCoCoChecker;
import schemalang._cocos.SchemaLangCocoFactory;
import schemalang.exception.SchemaLangTechnicalException;
import schematypes.TypeIdentifier;
import schematypes._ast.ASTObjectType;
import schematypes._ast.ASTSchemaType;

import java.util.*;
import java.util.stream.Collectors;

public class SchemaLangSymbolTableCreator extends SchemaLangSymbolTableCreatorTOP {

    private static Set<String> visitedTypes = Sets.newHashSet();

    private final SchemaLangCoCoChecker checker = SchemaLangCocoFactory.getCheckerWithAllCoCos();

    private GlobalScope globalScope;

    private Map<String, Set<SchemaDefinitionSymbol>> schemaImportsMap = Maps.newHashMap();

    private String compilationUnitPackage;

    public SchemaLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public SchemaLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    @Override
    public void visit(ASTSchemaLangCompilationUnit compilationUnit) {
        super.visit(compilationUnit);

        ASTSchemaDefinition schemaLangDefinition = compilationUnit.getSchemaDefinition();

        // checker.checkAll(compilationUnit);

        Optional<? extends MutableScope> mutableScope = currentScope();
        mutableScope.ifPresent(scope -> globalScope = (GlobalScope) scope);

        compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackageList());
        List<ImportStatement> imports = Lists.newArrayList();
        Set<SchemaDefinitionSymbol> importedSchemas = Sets.newHashSet();

        for (ASTImportStatement astImportStatement : compilationUnit.getImportStatementList()) {
            String qualifiedImport = Names.getQualifiedName(astImportStatement.getQualifiedName().getPartList());
            ImportStatement importStatement = new ImportStatement(qualifiedImport, astImportStatement.isStar());
            imports.add(importStatement);

            Optional<SchemaDefinitionSymbol> schemaLangDefinitionSymbolOpt = globalScope.resolve(qualifiedImport, SchemaDefinitionSymbol.KIND);
            if (!schemaLangDefinitionSymbolOpt.isPresent()) {
                throw new SchemaLangTechnicalException(String.format("No schema named '%s' for import.", qualifiedImport));
            }
            importedSchemas.add(schemaLangDefinitionSymbolOpt.get());
        }

        schemaImportsMap.put(schemaLangDefinition.getName(), importedSchemas);
        Optional<String> fileNameOpt = compilationUnit.get_SourcePositionStart().getFileName();
        if (fileNameOpt.isPresent()) {
            String fullPath = fileNameOpt.get();
            schemaLangDefinition.setFileName(FilenameUtils.getBaseName(fullPath));
        }

        ArtifactScope artifactScope = new SchemaLangArtifactScope(compilationUnitPackage, imports);
        putOnStack(artifactScope);
    }

    @Override
    public void endVisit(ASTSchemaLangCompilationUnit ast) {
        super.endVisit(ast);
        visitedTypes.clear();
    }

    @Override
    public void endVisit(ASTSchemaDefinition ast) {
        super.endVisit(ast);

        if (globalScope == null) return;

        SchemaDefinitionSymbol symbol = ast.getSchemaDefinitionSymbol();
        Optional<ASTSchemaExtendUsage> schemaExtendUsageOpt = ast.getSchemaExtendUsageOpt();
        if (schemaExtendUsageOpt.isPresent()) {
            ASTSchemaExtendUsage schemaExtendUsage = schemaExtendUsageOpt.get();
            String join;
            Optional<SchemaDefinitionSymbol> superSchema;
            for (ASTQualifiedName qualifiedName : schemaExtendUsage.getSuperSchemaList()) {
                join = Joiner.on('.').join(qualifiedName.getPartList());

                if (visitedTypes.contains(join)) {
                    continue;
                }
                visitedTypes.add(join);

                superSchema = globalScope.resolve(join, SchemaDefinitionSymbol.KIND); // If package ist defined, then the schema cannot be resolved
                if (superSchema.isPresent()) {
                    SchemaDefinitionSymbol superSchemaSymbol = superSchema.get();
                    symbol.addSuperSchema(superSchemaSymbol);
                    ast.addSuperSchema(superSchemaSymbol.getSchemaDefinitionNode().get());
                    setParentSymbols(ast, superSchemaSymbol.getSchemaDefinitionNode().get());
                }
            }
        }

        if (schemaImportsMap.containsKey(ast.getName())) {
            Set<SchemaDefinitionSymbol> importedSchemas = schemaImportsMap.get(ast.getName());
            importedSchemas.stream().forEach(symbol::addImportedSchema);
        }
    }

    private void setParentSymbols(ASTSchemaDefinition schema, ASTSchemaDefinition superSchema) {
        List<ASTComplexPropertyDefinition> schemaComplexPropertyDefinitions = schema.getComplexPropertyDefinitions();
        List<ASTComplexPropertyDefinition> superSchemaComplexPropertyDefinitions = superSchema.getComplexPropertyDefinitions();

        if (schemaComplexPropertyDefinitions == null || superSchemaComplexPropertyDefinitions == null) {
            return;
        }

        for (ASTComplexPropertyDefinition schemaComplexPropertyDefinition : schemaComplexPropertyDefinitions) {
            Optional<ASTComplexPropertyDefinition> superPropertyDefinitionOpt = superSchemaComplexPropertyDefinitions.stream()
                    .filter(p -> p.getName().equals(schemaComplexPropertyDefinition.getName())).findFirst();
            if (!superPropertyDefinitionOpt.isPresent()) {
                continue;
            }
            ASTComplexPropertyDefinition superPropertyDefinition = superPropertyDefinitionOpt.get();
            schemaComplexPropertyDefinition.setParentComplexProperty(superPropertyDefinition);

            ComplexPropertyDefinitionSymbol superPropertyDefinitionSymbol = superPropertyDefinition.getComplexPropertyDefinitionSymbol();
            ComplexPropertyDefinitionSymbol schemaComplexPropertyDefinitionSymbol = schemaComplexPropertyDefinition.getComplexPropertyDefinitionSymbol();
            schemaComplexPropertyDefinitionSymbol.setParentComplexProperty(superPropertyDefinitionSymbol);
        }
    }

    @Override
    public void endVisit(ASTTypedDeclaration node) {
        super.endVisit(node);
        Optional<TypedDeclarationSymbol> symbolOpt = node.getTypedDeclarationSymbolOpt();
        if (!symbolOpt.isPresent()) {
            return;
        }

        TypedDeclarationSymbol symbol = symbolOpt.get();
        ASTSchemaType schemaType = node.getType();
        symbol.setSchemaLangType(schemaType.getSchemaLangType());
        if (TypeIdentifier.OBJECT.equals(schemaType.getSchemaLangType())) {
            ASTObjectType customType = (ASTObjectType) schemaType;
            symbol.setTypeName(customType.getGenericType());
        }
        symbol.setType(schemaType);
    }

    @Override
    public void endVisit(ASTEnumeratedDeclaration ast) {
        super.endVisit(ast);
        Optional<EnumeratedDeclarationSymbol> schemaEnumAttributeSymbolOpt = ast.getEnumeratedDeclarationSymbolOpt();
        if (!schemaEnumAttributeSymbolOpt.isPresent()) {
            return;
        }
        EnumeratedDeclarationSymbol schemaEnumAttributeSymbol = schemaEnumAttributeSymbolOpt.get();

        Optional<ASTSignedLiteral> initialOpt = ast.getInitialOpt();
        initialOpt.ifPresent(astSignedLiteral -> schemaEnumAttributeSymbol.setDefaultValue(getValue(astSignedLiteral)));

        List<ASTSchemaConstant> enumsList = ast.getEnumsList();
        Collection<String> enumValuesAsString = enumsList.stream()
                .map(ASTSchemaConstant::getName)
                .collect(Collectors.toList());
        if (!enumValuesAsString.isEmpty()) {
            schemaEnumAttributeSymbol.setEnumerationValues(enumValuesAsString);
        }
    }

    @Override
    public void endVisit(ASTComplexPropertyDefinition ast) {
        super.endVisit(ast);
        List<ASTComplexPropertyValueDefinition> complexPropertyValueDefinitions = ast.getComplexPropertyValueDefinitionList();
        if (complexPropertyValueDefinitions == null) {
            return;
        }

        ComplexPropertyDefinitionSymbol complexPropertyDefinitionSymbol = ast.getComplexPropertyDefinitionSymbol();
        for (ASTComplexPropertyValueDefinition complexPropertyValueDefinition : complexPropertyValueDefinitions) {
            complexPropertyDefinitionSymbol.addPropertyValueDefinition(complexPropertyValueDefinition.getComplexPropertyValueDefinitionSymbol());
        }
    }

    @Override
    public void endVisit(ASTComplexPropertyValueDefinition ast) {
        super.endVisit(ast);
        List<ASTSchemaMember> schemaMembers = ast.getSchemaMemberList();
        if (schemaMembers == null) {
            return;
        }

        ComplexPropertyValueDefinitionSymbol symbol = ast.getComplexPropertyValueDefinitionSymbol();
        for (ASTSchemaMember schemaMember : schemaMembers) {
            if (SchemaMemberType.BASIC.equals(schemaMember.getSchemaMemberType())) {
                ASTTypedDeclaration basicSchemaProperty = (ASTTypedDeclaration) schemaMember;
                symbol.addBasicSchemaProperty(basicSchemaProperty.getTypedDeclarationSymbol());
            }
        }
    }

    @Override
    public void endVisit(ASTNestedSchemaEnumLinkDefinition ast) {
        super.endVisit(ast);
        Optional<NestedSchemaEnumLinkDefinitionSymbol> nestedSchemaEnumLinkDefinitionSymbolOpt = ast.getNestedSchemaEnumLinkDefinitionSymbolOpt();
        if (!nestedSchemaEnumLinkDefinitionSymbolOpt.isPresent()) {
            return;
        }

        NestedSchemaEnumLinkDefinitionSymbol linkDefinitionSymbol = nestedSchemaEnumLinkDefinitionSymbolOpt.get();
        Optional<ASTSignedLiteral> initialOpt = ast.getInitialOpt();
        initialOpt.ifPresent(astSignedLiteral -> linkDefinitionSymbol.setDefaultValue(getValue(astSignedLiteral)));

        for (ASTSchemaLink schemaLink : ast.getLinksList()) {
            if (schemaLink.getSchemaLinkReferenceOpt().isPresent()) {
                linkDefinitionSymbol.addSchemaLink(schemaLink.getName(), schemaLink.getSchemaLinkReferenceOpt().get().getSchema().getValue());
            } else {
                linkDefinitionSymbol.addSchemaLink(schemaLink.getName(), schemaLink.getName());
            }
        }

        SchemaDefinitionScope enclosingScope = (SchemaDefinitionScope) ast.getEnclosingScope();
        Optional<SchemaDefinitionSymbol> schemaLangDefinitionSymbolOpt =
                (Optional<SchemaDefinitionSymbol>) enclosingScope.getSpanningSymbol();
        SchemaDefinitionSymbol schemaDefinitionSymbol = schemaLangDefinitionSymbolOpt.get();
        schemaDefinitionSymbol.addSchemaEnumLinkDefinitionSymbol(ast.getNestedSchemaEnumLinkDefinitionSymbol());
    }

    @Override
    protected ComplexPropertyDefinitionSymbol create_ComplexPropertyDefinition(ASTComplexPropertyDefinition ast) {

        ASTComplexPropertyValues complexAttributeValues = ast.getComplexPropertyValues();
        if (complexAttributeValues == null || complexAttributeValues.getValuesList() == null || complexAttributeValues.getValuesList().isEmpty()) {
            return super.create_ComplexPropertyDefinition(ast);
        }

        Collection<String> validValues = Lists.newArrayList();
        for (ASTSchemaConstant schemaConstant : complexAttributeValues.getValuesList()) {
            validValues.add(schemaConstant.getName());
        }
        return new ComplexPropertyDefinitionSymbol(ast.getName(), validValues, ast.isOverride());
    }

    @Override
    public void endVisit(ASTReferenceModel node) {
        super.endVisit(node);
        List<ASTQualifiedName> referenceModelNames = node.getReferenceModelList();
        if (referenceModelNames == null || referenceModelNames.isEmpty()) {
            return;
        }

        Collection<ReferenceModel> referenceModels = Lists.newArrayList();
        for (ASTQualifiedName referenceModelName : referenceModelNames) {
            String qualifiedName = Joiner.on('.').join(referenceModelName.getPartList());

            Optional<EMAComponentSymbol> emaComponentOpt = globalScope.resolve(qualifiedName, EMAComponentSymbol.KIND);
            if (!emaComponentOpt.isPresent()) {
                SchemaDefinitionScope schemaDefinitionScope = (SchemaDefinitionScope) node.getEnclosingScope();
                SchemaDefinitionSymbol schemaDefinitionSymbol =
                        (SchemaDefinitionSymbol) schemaDefinitionScope.getSpanningSymbol().get();
                throw new SchemaLangTechnicalException(String.format("Reference model '%s' in schema definition '%s' " +
                        "could not be resolved.", qualifiedName, schemaDefinitionSymbol.getName()));
            }
            ReferenceModel referenceModel = new ReferenceModel();
            referenceModel.setEMAComponent(emaComponentOpt.get());
            referenceModels.add(referenceModel);
        }
        ReferenceModelSymbol symbol = node.getReferenceModelSymbol();
        SchemaDefinitionScope enclosingScope = (SchemaDefinitionScope) symbol.getEnclosingScope();
        Optional<? extends ScopeSpanningSymbol> spanningSymbolOpt = enclosingScope.getSpanningSymbol();
        SchemaDefinitionSymbol schemaDefinitionSymbol = (SchemaDefinitionSymbol) spanningSymbolOpt.get();
        schemaDefinitionSymbol.setReferenceModels(referenceModels);
    }

    @Override
    public void endVisit(ASTRequiresRule node) {
        super.endVisit(node);

        RequiresRuleSymbol requiresRuleSymbol = node.getRequiresRuleSymbol();
        requiresRuleSymbol.setDependencies(node.getDependenciesList());
    }

    public Object getValue(ASTSignedLiteral signedLiteral) {
        if (signedLiteral instanceof ASTTypelessLiteral) {
            ASTTypelessLiteral typelessLiteral = (ASTTypelessLiteral) signedLiteral;
            return typelessLiteral.getValue();
        }
        Log.warn(String.format("Type '%s' is not supported.", signedLiteral));
        return null;
    }
}