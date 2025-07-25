package schemalang.validation.cocos;

import com.google.common.base.Joiner;
import com.google.common.collect.Lists;
import conflang._ast.ASTConfigurationEntry;
import conflang._ast.ASTNestedConfigurationEntry;
import conflang._symboltable.ConfigurationEntry;
import conflang._symboltable.ConfigurationEntrySymbol;
import conflang._symboltable.NestedConfigurationEntrySymbol;
import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.ScopeSpanningSymbol;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTRequiresRule;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaMember;
import schemalang._ast.ASTTypedDeclaration;
import schemalang._symboltable.*;
import schemalang.validation.SchemaViolation;
import schemalang.validation.ValidationHelpers;
import schematypes._ast.*;

import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;
import java.util.stream.Stream;

import static schemalang.ErrorCodes.*;
import static schemalang.validation.TypeCompatibility.isValueCompatibleWithType;
import static schemalang.validation.ValidationHelpers.*;

public abstract class AbstractSchemaValidator {

    protected List<SchemaDefinitionSymbol> schemaDefinitionSymbols;
    private final List<SchemaViolation> schemaViolations;
    private boolean logError = false;

    public AbstractSchemaValidator(List<SchemaDefinitionSymbol> schemaDefinitionSymbols,
                                   List<SchemaViolation> schemaViolations) {
        this.schemaDefinitionSymbols = schemaDefinitionSymbols;
        this.schemaViolations = schemaViolations;
    }

    protected void addSchemaViolation(SchemaViolation schemaViolation) {
        schemaViolations.add(schemaViolation);
    }

    protected void checkTypedDeclaration(ASTConfigurationEntry configurationEntry, TypedDeclarationSymbol basicSchemaProperty) {

        ASTSignedLiteral value = configurationEntry.getValue();
        ASTSchemaType type = basicSchemaProperty.getType();

        // Use polymorphism instead of instanceof -> open/closed principle -> strategy pattern

        boolean isValid = isValueCompatibleWithType(value, type);
        if (!isValid) {
            String errorMessage = String.format(ERROR_MSG_SL_11C, configurationEntry.getName(), basicSchemaProperty.getType());
            addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_11C, errorMessage, Joiner.on(',').join(getAllSchemaNames())));
            Log.error(ERROR_CODE_SL_11C.concat(errorMessage), configurationEntry.get_SourcePositionStart());
        }

        if (type.isTypeWithRange()) {
            ASTTypeWithRange typeWithRange = (ASTTypeWithRange) type;
            Optional<ASTRange> rangeOpt = typeWithRange.getRangeOpt();
            if (!rangeOpt.isPresent()) {
                return;
            }

            ASTRange astRange = rangeOpt.get();
            boolean isInRange = isInRange(value, astRange);
            if (isValid && !isInRange) {
                String errorMessage = String.format(ERROR_MSG_SL_14C, configurationEntry.getName(), astRange);
                addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_14C, errorMessage, Joiner.on(',').join(getAllSchemaNames())));
                Log.error(ERROR_CODE_SL_14C.concat(errorMessage), configurationEntry.get_SourcePositionStart());
            }

            Optional<ASTNumberWithInf> stepResolutionOpt = astRange.getStepOpt();
            if (stepResolutionOpt.isPresent() && isValid && isInRange) {
                ASTNumberWithInf stepResolution = stepResolutionOpt.get();
                boolean isOfScale = isOfScale(value, stepResolution);
                if (!isOfScale) {
                    String errorMessage = String.format(ERROR_MSG_SL_15C, configurationEntry.getName(), stepResolution);
                    addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_15C, errorMessage, Joiner.on(',').join(getAllSchemaNames())));
                    Log.error(ERROR_CODE_SL_15C.concat(errorMessage), configurationEntry.get_SourcePositionStart());
                }
            }
        }
    }

    protected void checkEnumAttributeDefinition(ASTConfigurationEntry configurationEntry,
                                                EnumeratedDeclarationSymbol enumSchemaProperty) {

        ASTSignedLiteral signedLiteral = configurationEntry.getValue();
        if (!isConstant(signedLiteral)) {
            throw new RuntimeException("Wrong type for enum. Enum constants must be typeless.");
        }

        ASTTypelessLiteral enumLiteral = (ASTTypelessLiteral) signedLiteral;
        boolean valueExists = enumSchemaProperty.isValidEnumeration(enumLiteral.getValue());

        if (!valueExists) {
            String errorMessage = String.format(ERROR_MSG_SL_12C, enumLiteral.getValue(), enumSchemaProperty.getName(), Joiner.on(", ").join(enumSchemaProperty.getEnumerations()));
            addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_12C, errorMessage, Joiner.on(',').join(getAllSchemaNames())));
            Log.error(ERROR_CODE_SL_12C.concat(errorMessage), configurationEntry.get_SourcePositionStart());
        }
    }

    protected Optional<TypedDeclarationSymbol> findBasicPropertyInObjectTypeDefinitions(Scope enclosingScope, ConfigurationEntrySymbol symbol) {
        Optional<TypedDeclarationSymbol> simpleAttributeSymbolOpt = Optional.empty();
        Optional<? extends ScopeSpanningSymbol> spanningSymbolOpt = enclosingScope.getSpanningSymbol();
        Optional<ComplexPropertyDefinitionSymbol> complexAttributeSymbolOpt = Optional.empty();
        ASTTypelessLiteral value = null;

        if (spanningSymbolOpt.isPresent()) {
            ScopeSpanningSymbol spanningSymbol = spanningSymbolOpt.get();
            if (NestedConfigurationEntrySymbol.KIND.isSame(spanningSymbol.getKind())) {
                simpleAttributeSymbolOpt = getTypedDeclaration(spanningSymbol.getName());
                NestedConfigurationEntrySymbol nestedConfigurationEntrySymbol = (NestedConfigurationEntrySymbol) spanningSymbol;
                ASTNestedConfigurationEntry astNode = (ASTNestedConfigurationEntry) nestedConfigurationEntrySymbol.getAstNode().get();
                value = (ASTTypelessLiteral) astNode.getValue();
            }
        }

        Optional<ASTObjectType> complexType = Optional.empty();
        if (simpleAttributeSymbolOpt.isPresent()) {
            TypedDeclarationSymbol simpleAttributeSymbol = simpleAttributeSymbolOpt.get();
            ASTTypedDeclaration schemaSimpleAttributeNode = simpleAttributeSymbol.getTypedDeclarationNode().get();
            ASTSchemaType type = schemaSimpleAttributeNode.getType();
            if (type instanceof ASTObjectType) {
                complexType = Optional.of((ASTObjectType) type);
            }
        }

        if (complexType.isPresent()) {
            String genericType = complexType.get().getGenericType();
            complexAttributeSymbolOpt = getObjectDefinition(genericType);
        }

        Optional<TypedDeclarationSymbol> attributeDefinitionSymbolOpt;
        if (complexAttributeSymbolOpt.isPresent()) {
            ComplexPropertyDefinitionSymbol definitionSymbol = complexAttributeSymbolOpt.get();
            attributeDefinitionSymbolOpt = definitionSymbol.getSchemaDefinition(symbol.getName(), value.getValue());
            if (attributeDefinitionSymbolOpt.isPresent()) {
                return attributeDefinitionSymbolOpt;
            }
        }

        if (complexAttributeSymbolOpt.isPresent()) {
            ComplexPropertyDefinitionSymbol definitionSymbol = complexAttributeSymbolOpt.get();
            Scope spannedScope = definitionSymbol.getSpannedScope();
            List<? extends Scope> subScopes = spannedScope.getSubScopes();
            Optional<ComplexPropertyValueDefinitionSymbol> valueDefinitionSymbol = Optional.empty();
            for (Scope subScope : subScopes) {
                if (subScope.getSpanningSymbol().get().getName().equals(value.getValue())) {
                    valueDefinitionSymbol = (Optional<ComplexPropertyValueDefinitionSymbol>) subScope.getSpanningSymbol();
                }
            }

            if (valueDefinitionSymbol.isPresent()) {
                Scope subScope = valueDefinitionSymbol.get().getSpannedScope();
                simpleAttributeSymbolOpt = subScope.resolve(symbol.getName(), TypedDeclarationSymbol.KIND);
            }
        }

        if (simpleAttributeSymbolOpt.isPresent()) {
            TypedDeclarationSymbol typedDeclarationSymbol = simpleAttributeSymbolOpt.get();
            if (!typedDeclarationSymbol.getName().equals(symbol.getName())) {
                return Optional.empty();
            }
        }
        return simpleAttributeSymbolOpt;
    }

    protected Optional<EnumeratedDeclarationSymbol> findEnumPropertyInObjectTypeDefinitions(Scope enclosingScope, ConfigurationEntrySymbol symbol) {
        Optional<TypedDeclarationSymbol> simpleAttributeSymbolOpt = Optional.empty();
        Optional<? extends ScopeSpanningSymbol> spanningSymbolOpt = enclosingScope.getSpanningSymbol();
        Optional<ComplexPropertyDefinitionSymbol> complexAttributeSymbolOpt = Optional.empty();
        ASTTypelessLiteral value = null;

        if (spanningSymbolOpt.isPresent()) {
            ScopeSpanningSymbol spanningSymbol = spanningSymbolOpt.get();
            if (NestedConfigurationEntrySymbol.KIND.isSame(spanningSymbol.getKind())) {
                simpleAttributeSymbolOpt = getTypedDeclaration(spanningSymbol.getName());
                NestedConfigurationEntrySymbol nestedConfigurationEntrySymbol = (NestedConfigurationEntrySymbol) spanningSymbol;
                ASTNestedConfigurationEntry astNode = (ASTNestedConfigurationEntry) nestedConfigurationEntrySymbol.getAstNode().get();
                value = (ASTTypelessLiteral) astNode.getValue();
            }
        }

        Optional<ASTObjectType> complexType = Optional.empty();
        if (simpleAttributeSymbolOpt.isPresent()) {
            TypedDeclarationSymbol simpleAttributeSymbol = simpleAttributeSymbolOpt.get();
            ASTTypedDeclaration schemaSimpleAttributeNode = simpleAttributeSymbol.getTypedDeclarationNode().get();
            ASTSchemaType type = schemaSimpleAttributeNode.getType();
            if (type instanceof ASTObjectType) {
                complexType = Optional.of((ASTObjectType) type);
            }
        }

        if (complexType.isPresent()) {
            String genericType = complexType.get().getGenericType();
            complexAttributeSymbolOpt = getObjectDefinition(genericType);
        }

        Optional<EnumeratedDeclarationSymbol> enumDefinitionSymbolOpt = Optional.empty();
        if (complexAttributeSymbolOpt.isPresent()) {
            ComplexPropertyDefinitionSymbol definitionSymbol = complexAttributeSymbolOpt.get();
            Scope spannedScope = definitionSymbol.getSpannedScope();
            enumDefinitionSymbolOpt = spannedScope.resolve(symbol.getName(), EnumeratedDeclarationSymbol.KIND);
            if (enumDefinitionSymbolOpt.isPresent()) {
                return enumDefinitionSymbolOpt;
            }
        }

        if (complexAttributeSymbolOpt.isPresent()) {
            ComplexPropertyDefinitionSymbol definitionSymbol = complexAttributeSymbolOpt.get();
            Scope spannedScope = definitionSymbol.getSpannedScope();
            List<? extends Scope> subScopes = spannedScope.getSubScopes();
            Optional<ComplexPropertyValueDefinitionSymbol> valueDefinitionSymbol = Optional.empty();
            for (Scope subScope : subScopes) {
                if (subScope.getSpanningSymbol().get().getName().equals(value.getValue())) {
                    valueDefinitionSymbol = (Optional<ComplexPropertyValueDefinitionSymbol>) subScope.getSpanningSymbol();
                }
            }

            if (valueDefinitionSymbol.isPresent()) {
                Scope subScope = valueDefinitionSymbol.get().getSpannedScope();
                enumDefinitionSymbolOpt = subScope.resolve(symbol.getName(), EnumeratedDeclarationSymbol.KIND);
            }
        }
        return enumDefinitionSymbolOpt;
    }

    protected List<String> getAllSchemaNames() {
        List<String> names = Lists.newArrayList();
        for (SchemaDefinitionSymbol schemaDefinitionSymbol : schemaDefinitionSymbols) {
            names.add(schemaDefinitionSymbol.getName());
        }
        return names;
    }

    protected Optional<ComplexPropertyDefinitionSymbol> getObjectDefinition(String name) {
        Optional<ComplexPropertyDefinitionSymbol> attributeDefinitionSymbol = Optional.empty();
        for (SchemaDefinitionSymbol schemaDefinitionSymbol : schemaDefinitionSymbols) {
            attributeDefinitionSymbol = schemaDefinitionSymbol.getAttributeEntryOfKindComplex(name);
            if (attributeDefinitionSymbol.isPresent()) {
                return attributeDefinitionSymbol;
            }
        }
        return attributeDefinitionSymbol;
    }

    protected Optional<TypedDeclarationSymbol> getTypedDeclaration(String name) {
        Optional<TypedDeclarationSymbol> attributeEntryOfKindSimple = Optional.empty();
        for (SchemaDefinitionSymbol schemaDefinitionSymbol : schemaDefinitionSymbols) {
            attributeEntryOfKindSimple = schemaDefinitionSymbol.getTypedDeclaration(name);
            if (attributeEntryOfKindSimple.isPresent()) {
                return attributeEntryOfKindSimple;
            }
        }
        return attributeEntryOfKindSimple;
    }

    protected Optional<EnumeratedDeclarationSymbol> getAttributeEntryOfKindEnum(String name) {
        Optional<EnumeratedDeclarationSymbol> attributeEntryOfKindSimple = Optional.empty();
        for (SchemaDefinitionSymbol schemaDefinitionSymbol : schemaDefinitionSymbols) {
            attributeEntryOfKindSimple = schemaDefinitionSymbol.getEnumeratedDeclaration(name);
            if (attributeEntryOfKindSimple.isPresent()) {
                return attributeEntryOfKindSimple;
            }
        }
        return attributeEntryOfKindSimple;
    }

    protected Optional<NestedSchemaEnumLinkDefinitionSymbol> getAttributeEntryOfKindEnumLinkDefinition(String name) {
        Optional<NestedSchemaEnumLinkDefinitionSymbol> attributeEntryOfKindSimple = Optional.empty();
        for (SchemaDefinitionSymbol schemaDefinitionSymbol : schemaDefinitionSymbols) {
            attributeEntryOfKindSimple = schemaDefinitionSymbol.getAttributeEntryOfKindEnumSchemaLinks(name);
            if (attributeEntryOfKindSimple.isPresent()) {
                return attributeEntryOfKindSimple;
            }
        }
        return attributeEntryOfKindSimple;
    }

    protected List<ASTSchemaMember> getAllRequiredProperties() {
        List<ASTSchemaMember> allRequiredProperties = Lists.newArrayList();
        Optional<ASTSchemaDefinition> schemaLangDefinitionNodeOpt;
        for (SchemaDefinitionSymbol schemaDefinitionSymbol : schemaDefinitionSymbols) {
            schemaLangDefinitionNodeOpt = schemaDefinitionSymbol.getSchemaDefinitionNode();
            allRequiredProperties.addAll(schemaLangDefinitionNodeOpt.get().getRequiredPropertyDefinitions());
        }
        return allRequiredProperties;
    }

    protected List<ASTRequiresRule> getAllRequiresRules() {
        List<ASTRequiresRule> allRequiresRules = Lists.newArrayList();
        Optional<ASTSchemaDefinition> schemaLangDefinitionNodeOpt;
        for (SchemaDefinitionSymbol schemaDefinitionSymbol : schemaDefinitionSymbols) {
            schemaLangDefinitionNodeOpt = schemaDefinitionSymbol.getSchemaDefinitionNode();
            allRequiresRules.addAll(schemaLangDefinitionNodeOpt.get().getRequiresRuleDefinitions());
        }
        return allRequiresRules;
    }

    protected boolean isDefinedInReferenceModel(String componentName) {

        Collection<EMAComponentSymbol> referenceModels = getReferenceModels(schemaDefinitionSymbols);
        for (EMAComponentSymbol referenceModel : referenceModels) {
            if (referenceModel.getSubComponent(componentName).isPresent()) {
                return true;
            }
            if (isDefinedAsParameterInReferenceModel(componentName, referenceModel)) return true;
        }
        return false;
    }
    protected boolean isNestedEntryDefinedInReferenceModels(NestedConfigurationEntrySymbol symbol) {
        Collection<EMAComponentSymbol> linkedReferenceModels = ValidationHelpers.getReferenceModels(schemaDefinitionSymbols);
        List<ConfigurationEntry> allConfigurationEntries = symbol.getAllConfigurationEntries();
        String entryName = symbol.getName();
        for (EMAComponentSymbol referenceModel :
                linkedReferenceModels) {
            //is nested entry defined in reference model
            if (isConformNestedEntryToInstance(allConfigurationEntries, entryName, referenceModel)) {
                return true;
            }
        }
        return false;
    }

    private static boolean isConformNestedEntryToInstance(List<ConfigurationEntry> allConfigurationEntries, String entryName, EMAComponentSymbol referenceModel) {
        final EMAComponentInstantiationSymbol instance = referenceModel.getSubComponent(entryName).orElse(null);
        final ASTComponent astNode = (ASTComponent) referenceModel.getAstNode().orElseThrow(IllegalStateException::new);
        if (instance == null)
            return false;
        //now check if parameters are conform
        final Predicate<ASTComponent> filterPredicateToFindNestedEntryName = astComponent -> astComponent.getName().equals(instance.getComponentType().getReferencedSymbol().getName());
        final Stream<ASTComponent> filteredSubComponents = astNode.getInnerComponents().stream().filter(filterPredicateToFindNestedEntryName);
        final ASTComponent componentForInstance = filteredSubComponents.findFirst().orElseThrow(IllegalStateException::new);
        final List<ASTParameter> componentParameters = componentForInstance.getParameterList();
        final Stream<ASTParameter> emaVariableStream = componentParameters.stream().filter(componentParameter-> hasConfigurationEntriesParameter(allConfigurationEntries, componentParameter));
        return emaVariableStream.count() == allConfigurationEntries.size();
    }

    private static boolean hasConfigurationEntriesParameter(List<ConfigurationEntry> allConfigurationEntries, ASTParameter parameter) {
        for (ConfigurationEntry configurationEntry :
                allConfigurationEntries) {
            if (configurationEntry.getName().equals(parameter.getNameWithArray().getName()))
                return true;
        }
        return false;
    }

    private static boolean isDefinedAsParameterInReferenceModel(final String entryName, final EMAComponentSymbol referenceModel) {
        final ASTComponent astReferenceModel = (ASTComponent) referenceModel.getAstNode().orElseThrow(IllegalStateException::new);
        final List<ASTComponent> innerComponents = astReferenceModel.getInnerComponents();
        for (ASTComponent astComponent : innerComponents
        ) {
            long numOfMatchedParameters = astComponent.getParameterList().stream().filter(astParameter -> astParameter.getNameWithArray().getName().equals(entryName)).count();
            if (numOfMatchedParameters == 1) {
                return true;
            }
        }
        return false;
    }

    public boolean isLogError() {
        return logError;
    }

    public void setLogError(boolean logError) {
        this.logError = logError;
    }

    protected void logError(ASTNode node, String errorMessage) {
        if (isLogError()) {
            Log.error(errorMessage, node.get_SourcePositionStart());
        }
    }
}