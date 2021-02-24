package schemalang.validation;

import com.google.common.base.Strings;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import conflang._symboltable.ConfigurationEntry;
import conflang._symboltable.ConfigurationSymbol;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._cocos.SchemaLangCoCoChecker;
import schemalang._cocos.SchemaLangCocoFactory;
import schemalang._symboltable.NestedSchemaEnumLinkDefinitionSymbol;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang._symboltable.SchemaLangLanguage;
import schemalang.exception.SchemaLangTechnicalException;
import schemalang.validation.exception.SchemaLangException;
import schemalang.validation.exception.SchemaLangValidationException;
import schemalang.validation.model.ArchitectureComponent;

import java.util.*;

public class SchemaLangValidator {

    public static List<SchemaDefinitionSymbol> resolveSchemaHierarchy(SchemaDefinitionSymbol schemaDefinitionSymbol,
                                                                            ConfigurationSymbol configurationSymbol,
                                                                            ModelPath modelPath) throws SchemaLangException {
        ModelingLanguageFamily family = new ModelingLanguageFamily();
        family.addModelingLanguage(new SchemaLangLanguage());
        family.addModelingLanguage(new EmbeddedMontiArcLanguage());
        GlobalScope globalScope = new GlobalScope(modelPath, family);

        List<SchemaDefinitionSymbol> schemas = Lists.newArrayList();
        if ((schemaDefinitionSymbol.getSchemaEnumLinkDefinitionSymbols() != null && !schemaDefinitionSymbol.getSchemaEnumLinkDefinitionSymbols().isEmpty())) {
            schemas.addAll(collectAllSchemas(configurationSymbol, schemaDefinitionSymbol, globalScope));
        } else {
            schemas.add(schemaDefinitionSymbol);
        }
        return schemas;
    }

    public static List<SchemaViolation> validateConfiguration(SchemaDefinitionSymbol schemaDefinitionSymbol,
                                                              ConfigurationSymbol configurationSymbol) throws SchemaLangException {

        return SchemaDefinitionValidator.validateConfiguration(Lists.newArrayList(schemaDefinitionSymbol), configurationSymbol);
    }

    public static List<SchemaViolation> validateConfiguration(List<SchemaDefinitionSymbol> schemaDefinitionSymbols,
                                                              ConfigurationSymbol configurationSymbol) throws SchemaLangException {

        return SchemaDefinitionValidator.validateConfiguration(schemaDefinitionSymbols, configurationSymbol);
    }

    public static Collection<Violation> validate(ConfigurationSymbol configurationSymbol, String schemaName,
                                                 Map<String, ArchitectureComponent> componentMap,
                                                 ModelPath modelPath) throws SchemaLangException {

        ModelingLanguageFamily family = new ModelingLanguageFamily();
        family.addModelingLanguage(new SchemaLangLanguage());
        family.addModelingLanguage(new EmbeddedMontiArcLanguage());
        GlobalScope globalScope = new GlobalScope(modelPath, family);

        List<SchemaDefinitionSymbol> schemas = Lists.newArrayList();
        schemas.addAll(SchemaLangValidator.getSchemas(configurationSymbol, schemaName, modelPath, globalScope));

        List<SchemaViolation> schemaViolations =
                SchemaDefinitionValidator.validateConfiguration(schemas, configurationSymbol);
        List<ReferenceModelViolation> referenceModelViolations =
                ReferenceModelValidator.validate(schemas, configurationSymbol, componentMap);

        Collection<Violation> violations = Lists.newArrayList();
        violations.addAll(schemaViolations);
        violations.addAll(referenceModelViolations);
        return violations;
    }

    private static Collection<? extends SchemaDefinitionSymbol> getSchemas(ConfigurationSymbol configurationSymbol,
                                                                           String schema, ModelPath modelPath,
                                                                           GlobalScope schemaLangGlobalScope) throws SchemaLangException {

        List<SchemaDefinitionSymbol> schemas = Lists.newArrayList();
        if (modelPath == null) {
            throw new SchemaLangException("The model path must not be null.");
        }

        if (Strings.isNullOrEmpty(schema)) {
            throw new SchemaLangException("Either a concrete schema or a schema with schema links must be provided.");
        }

        Optional<SchemaDefinitionSymbol> schemaLangDefinitionSymbolOpt =
                schemaLangGlobalScope.resolve(schema, SchemaDefinitionSymbol.KIND);
        if (!schemaLangDefinitionSymbolOpt.isPresent()) {
            throw new SchemaLangException("Schema '" + schema + "' could not be resolved in model path " + modelPath.toString());
        }

        SchemaDefinitionSymbol schemaDefinitionSymbol = schemaLangDefinitionSymbolOpt.get();
        checkCocos(schemaDefinitionSymbol);
        if ((schemaDefinitionSymbol.getSchemaEnumLinkDefinitionSymbols() != null && !schemaDefinitionSymbol.getSchemaEnumLinkDefinitionSymbols().isEmpty())) {
            schemas.addAll(collectAllSchemas(configurationSymbol, schemaDefinitionSymbol, schemaLangGlobalScope));
        } else {
            schemas.add(schemaDefinitionSymbol);
        }
        return schemas;
    }

    private static void checkCocos(SchemaDefinitionSymbol schemaDefinitionSymbol) {
        Optional<ASTSchemaDefinition> schemaDefinitionNode = schemaDefinitionSymbol.getSchemaDefinitionNode();
        if (!schemaDefinitionNode.isPresent()) {
            Log.error("AST node of schema definition "
                    + schemaDefinitionSymbol.getName() + " is not set. Cocos are not checked.");
            return;
        }
        SchemaLangCoCoChecker checkerWithAllCoCos = SchemaLangCocoFactory.getCheckerWithAllCoCos();
        checkerWithAllCoCos.checkAll(schemaDefinitionNode.get());
    }

    private static List<SchemaDefinitionSymbol> collectAllSchemas(ConfigurationSymbol configurationSymbol, SchemaDefinitionSymbol schemaDefinitionSymbol, GlobalScope globalScope) throws SchemaLangException {

        List<SchemaDefinitionSymbol> schemas = Lists.newArrayList();
        List<SchemaDefinitionSymbol> schemaDefinitionSymbols = Lists.newArrayList(schemaDefinitionSymbol);
        collectSchemas(configurationSymbol, schemaDefinitionSymbol, globalScope, schemaDefinitionSymbols);

        if (schemaDefinitionSymbols.isEmpty()) {
            throw new SchemaLangValidationException("No schemas could be loaded for validation.");
        }

        if (schemaDefinitionSymbols.size() > 1) {
            Iterator<SchemaDefinitionSymbol> iterator = schemaDefinitionSymbols.iterator();
            SchemaDefinitionSymbol first = iterator.next();
            SchemaDefinitionSymbol next;
            List<SchemaDefinitionSymbol> schemasTmp = Lists.newArrayList(schemaDefinitionSymbols);
            while (iterator.hasNext()) {
                next = iterator.next();
                if (first.isInHierarchy(next)) {
                    schemasTmp.remove(next);
                } else if (next.isInHierarchy(first)) {
                    schemasTmp.remove(first);
                }
            }
            schemaDefinitionSymbols = schemasTmp;
        }

        schemas.addAll(schemaDefinitionSymbols);
        return schemas;
    }

    private static void collectSchemas(ConfigurationSymbol configurationSymbol, SchemaDefinitionSymbol schemaDefinitionSymbol,
                                       GlobalScope globalScope, List<SchemaDefinitionSymbol> schemaDefinitionSymbols) throws SchemaLangValidationException {

        Set<NestedSchemaEnumLinkDefinitionSymbol> schemaEnumLinkDefinitionSymbols = schemaDefinitionSymbol.getSchemaEnumLinkDefinitionSymbols();
        Map<String, String> selectedSchemas = Maps.newHashMap();
        if (!schemaEnumLinkDefinitionSymbols.isEmpty()) {
            for (NestedSchemaEnumLinkDefinitionSymbol schemaLinkDefinitionSymbol : schemaEnumLinkDefinitionSymbols) {
                Optional<ConfigurationEntry> configurationEntryOpt = configurationSymbol.getConfigurationEntry(schemaLinkDefinitionSymbol.getName());
                if (configurationEntryOpt.isPresent()) {
                    ConfigurationEntry configurationEntry = configurationEntryOpt.get();
                    String key = (String) configurationEntry.getValue();
                    String value = schemaLinkDefinitionSymbol.getSchemaLink(key);
                    selectedSchemas.put(key, value);

                } else if (schemaDefinitionSymbol.hasDefault(schemaLinkDefinitionSymbol.getName())) {
                    Optional<String> key = schemaDefinitionSymbol.getDefaultValue(schemaLinkDefinitionSymbol.getName());
                    if (!key.isPresent()) {
                        throw new SchemaLangTechnicalException("This should have not happen.");
                    }
                    String value = schemaLinkDefinitionSymbol.getSchemaLink(key.get());
                    selectedSchemas.put(key.get(), value);
                }
            }
        }

        for (Map.Entry<String, String> entry : selectedSchemas.entrySet()) {
            Optional<SchemaDefinitionSymbol> schemaDefinitionSymbolOpt;
            if (Strings.isNullOrEmpty(entry.getValue())) {
                schemaDefinitionSymbolOpt = globalScope.resolve(entry.getKey(), SchemaDefinitionSymbol.KIND);
                if (!schemaDefinitionSymbolOpt.isPresent()) {
                    throw new SchemaLangValidationException("Validation aborted: no schema link defined for parameter value '" + entry.getKey() + "'.");
                }
            }

            schemaDefinitionSymbolOpt = globalScope.resolve(entry.getValue(), SchemaDefinitionSymbol.KIND);
            if (!schemaDefinitionSymbolOpt.isPresent()) {
                // backup: try with first letter capitalized
                String schemaNameCapitalized = StringUtils.capitalize(entry.getValue());
                schemaDefinitionSymbolOpt = globalScope.resolve(schemaNameCapitalized, SchemaDefinitionSymbol.KIND);
                if (!schemaDefinitionSymbolOpt.isPresent()) {
                    // backup: try with all letters capitalized
                    schemaNameCapitalized = StringUtils.upperCase(entry.getValue());
                    schemaDefinitionSymbolOpt = globalScope.resolve(schemaNameCapitalized, SchemaDefinitionSymbol.KIND);
                }
                if (!schemaDefinitionSymbolOpt.isPresent()) {
                    throw new SchemaLangValidationException("Validation aborted: no schema link defined for parameter value '" + entry.getKey() + "'.");
                }
            }
            checkCocos(schemaDefinitionSymbolOpt.get());
            schemaDefinitionSymbols.add(schemaDefinitionSymbolOpt.get());
            collectSchemas(configurationSymbol, schemaDefinitionSymbolOpt.get(), globalScope, schemaDefinitionSymbols);
        }
    }
}