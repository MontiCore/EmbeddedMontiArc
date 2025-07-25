package de.monticore.montipipes.util;

import com.google.common.collect.Iterables;
import conflang._ast.ASTConfigurationEntry;
import conflang._ast.ASTNestedConfigurationEntry;
import conflangliterals._ast.ASTComponentLiteral;
import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.mcliterals._ast.ASTSignedLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.monticore.montipipes.config.ExecutionScriptConfiguration;
import de.monticore.symboltable.CommonSymbol;
import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.tuple.Pair;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.StringJoiner;
import java.util.stream.Collectors;
import java.util.stream.Stream;

//used in freemarker templates
@SuppressWarnings("unused")
public class TemplateUtil {


    public static Pair<String, String> getModelDirNameAndValue(final Map<ExecutionScriptConfiguration, String> executionScriptConfigurations) {
        return Pair.of(ExecutionScriptConfiguration.MODEL_DIRECTORY.getConfigurationName(), executionScriptConfigurations.get(ExecutionScriptConfiguration.MODEL_DIRECTORY));
    }

    public ASTConfigurationEntry filterConfigurationEntriesForName(final List<ASTConfigurationEntry> configurationEntries, final String entryName) {
        return configurationEntries.stream().filter(entry -> entry.getName().equals(entryName)).findFirst().orElseThrow(IllegalStateException::new);
    }


    public String addKeywordsToMethodParameters(final String commaSeparatedMethodParameters) {
        final String[] params = commaSeparatedMethodParameters.split(",");
        return Arrays.stream(params).map(param -> param + '=' + param).collect(Collectors.joining(","));
    }

    public String getArgumentParameterPairsFromConfigurationEntryIfNested(final ASTConfigurationEntry astConfigurationEntry) {
        final StringJoiner stringJoiner = new StringJoiner(", ");
        if (astConfigurationEntry instanceof ASTNestedConfigurationEntry) {
            final List<ASTConfigurationEntry> configurationEntryList = ((ASTNestedConfigurationEntry) astConfigurationEntry).getConfigurationEntryList();
            configurationEntryList.forEach(configurationEntry -> {
                final String value = getValueFromEntry(configurationEntry.getValue());
                stringJoiner.add(configurationEntry.getName() + '=' + value);
            });
        }
        return stringJoiner.toString();
    }


    public String getArgumentParameterPairsForInitialisationCall(final EMAComponentInstanceSymbol emaComponentInstanceSymbol) {
        return getInputPortsFromEMAInstance(emaComponentInstanceSymbol).map(inputPort -> inputPort + "=" + inputPort).collect(Collectors.joining(", "));
    }

    public String getParametersForPipelineStep(final EMAComponentInstanceSymbol emaComponentInstanceSymbol) {
        final Stream<String> parameterNames = getParameterNamesFromEMAInstance(emaComponentInstanceSymbol);
        final Stream<String> inputPortsFromEMAInstance = getInputPortsFromEMAInstance(emaComponentInstanceSymbol);
        final Stream<String> implicitlyAddedParameters = getImplicitlyAddedParameters();
        return Stream.of(inputPortsFromEMAInstance, parameterNames, implicitlyAddedParameters).flatMap(i -> i).collect(Collectors.joining(", "));
    }

    public String getArgumentsForPipelineStep(final EMAComponentInstanceSymbol pipelineStep, final ASTConfigurationEntry stepConfiguration) {
        final String configurationArguments = getArgumentParameterPairsFromConfigurationEntryIfNested(stepConfiguration);
        final String inputPorts = getArgumentParameterPairsForInitialisationCall(pipelineStep);
        return Stream.of(inputPorts, configurationArguments).filter(StringUtils::isNotBlank).map(argument -> argument + ",").collect(Collectors.joining());
    }


    public String getOutputPortsFromEMAInstanceAsCommaSeperatedList(final EMAComponentInstanceSymbol emaComponentInstanceSymbol) {
        return getOutputPortsFromEMAInstance(emaComponentInstanceSymbol).collect(Collectors.joining(", "));
    }

    private Stream<String> getImplicitlyAddedParameters() {
        return Stream.of("schema_api");
    }

    private Stream<String> getParameterNamesFromEMAInstance(final EMAComponentInstanceSymbol emaComponentInstanceSymbol) {
        return emaComponentInstanceSymbol.getParameters().stream().map(EMAVariable::getName);
    }

    public Stream<String> getInputPortsFromEMAInstance(final EMAComponentInstanceSymbol emaComponentInstanceSymbol) {
        return emaComponentInstanceSymbol.getIncomingPortInstances().stream().map(CommonSymbol::getName);
    }

    public Stream<String> getOutputPortsFromEMAInstance(final EMAComponentInstanceSymbol emaComponentInstanceSymbol) {
        return emaComponentInstanceSymbol.getOutgoingPortInstances().stream().map(CommonSymbol::getName);
    }


    private static String getValueFromEntry(final ASTSignedLiteral configurationEntry) {
        if (configurationEntry instanceof ASTStringLiteral)
            return "'" + ((ASTStringLiteral) configurationEntry).getValue() + "'";
        if (configurationEntry instanceof ASTComponentLiteral)
            return String.format("%s()", getLastElementOfQualifiedName((ASTComponentLiteral) configurationEntry));
        if (configurationEntry instanceof ASTTypelessLiteral)
            return String.format("%s()", ((ASTTypelessLiteral) configurationEntry).getValue());
        throw new IllegalStateException("unexpected type of configuration value");
    }

    private static String getLastElementOfQualifiedName(final ASTComponentLiteral configurationEntry) {
        return Iterables.getLast(configurationEntry.getValue().getPartList());
    }

}
