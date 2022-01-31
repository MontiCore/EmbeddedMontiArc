package schemalang.validation.cocos;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import conflang._symboltable.ConfigurationEntry;
import conflang._symboltable.ConfigurationEntrySymbol;
import conflang._symboltable.ConfigurationSymbol;
import conflang._symboltable.NestedConfigurationEntrySymbol;
import de.monticore.ast.ASTNode;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberExpression;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolutionExpression;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTDimension;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.literals.literals._ast.ASTIntLiteral;
import de.monticore.literals.literals._ast.ASTNumericLiteral;
import de.monticore.numberunit._ast.ASTNumberWithInf;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;
import schemalang.exception.SchemaLangTechnicalException;
import schemalang.validation.ReferenceModelViolation;
import schemalang.validation.Violation;
import schemalang.validation.model.*;

import java.math.BigDecimal;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

import static schemalang.ErrorCodes.*;

public class ReferenceModelIsValid {

    public static ReferenceModelViolation validateReferenceModel(Map<String, ArchitectureComponent> componentMap,
                                                                 EMAComponentSymbol referenceModel,
                                                                 ConfigurationSymbol configuration,
                                                                 boolean logError) {

        List<Violation> violations = Lists.newArrayList();
        ReferenceModelViolation referenceModelViolation =
                new ReferenceModelViolation(referenceModel.getFullName(), violations);

        Set<String> componentsInComponentMap = Sets.newHashSet();
        Set<String> componentsNotInComponentMap = Sets.newHashSet();
        Map<String, EMAComponentInstantiationSymbol> referenceModelComponentsMap
                = referenceModel.getSubComponents().stream().collect(Collectors.toMap(
                        EMAComponentInstantiationSymbol::getName, Function.identity()));

        // Check which components are available
        for (String rmComponentName : referenceModelComponentsMap.keySet()) {
            if (!componentMap.containsKey(rmComponentName)) {
                componentsNotInComponentMap.add(rmComponentName);
                continue;
            }
            componentsInComponentMap.add(rmComponentName);
        }

        // Validate available components
        for (String rmComponent : componentsInComponentMap) {
            ArchitectureComponent realComponent = componentMap.get(rmComponent);
            EMAComponentInstantiationSymbol component = referenceModelComponentsMap.get(rmComponent);
            EMAComponentSymbolReference componentSymbolReference = component.getComponentType();
            Optional<EMAComponentSymbol> referenceModelComponent = componentSymbolReference.getReferencedComponent();
            if (!referenceModelComponent.isPresent()) {
                throw new SchemaLangTechnicalException("This should have not happen.");
            }
            validateComponentAgainstReferenceModel(realComponent, referenceModelComponent.get(),
                    violations, logError);
        }

        // Validate connectors
        validateConnectors(componentMap, referenceModel, logError, violations,
                componentsInComponentMap);

        if (componentsNotInComponentMap.isEmpty()) {
            return referenceModelViolation;
        }

        // Validate components against the training configuration
        checkComponentsInConfiguration(configuration, referenceModel, componentsNotInComponentMap,
                violations, logError);
        return referenceModelViolation;
    }

    private static void validateComponentAgainstReferenceModel(ArchitectureComponent realComponent,
                                                                  EMAComponentSymbol rmComponentSymbol,
                                                                  List<Violation> violations, boolean logError) {

        List<Port> realComponentInputs = realComponent.getIncomingPorts();
        List<EMAPortSymbol> rfInputs = rmComponentSymbol.getAllIncomingPorts();
        if (realComponentInputs.size() != rfInputs.size()) {
            String realInputPorts = realComponentInputs.stream().map(Port::getPortName).collect(
                    Collectors.joining(", "));
            String rmInputPorts = rfInputs.stream().map(EMAPortSymbol::getName).collect(
                    Collectors.joining(", "));
            String errorMessage = String.format(ERROR_MSG_TA_02C, realComponent.getFullName(),
                    realInputPorts, rmInputPorts);
            violations.add(Violation.create(ERROR_CODE_TA_02C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_02C.concat(errorMessage));
            }
        }

        List<Port> realComponentOutputs = realComponent.getOutgoingPorts();
        List<EMAPortSymbol> rfOutputs = rmComponentSymbol.getAllOutgoingPorts();
        if (realComponentOutputs.size() != rfOutputs.size()) {
            String realOutputPorts = realComponentOutputs.stream().map(Port::getPortName).collect(
                    Collectors.joining(", "));
            String rmOutputPorts = rfOutputs.stream().map(EMAPortSymbol::getName).collect(
                    Collectors.joining(", "));
            String errorMessage = String.format(ERROR_MSG_TA_03C, realComponent.getFullName(),
                    realOutputPorts, rmOutputPorts);
            violations.add(Violation.create(ERROR_CODE_TA_03C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_03C.concat(errorMessage));
            }
        }

        for (EMAPortSymbol rfInputPort : rfInputs) {
            Optional<Port> realIncomingPort = realComponent.getIncomingPort(rfInputPort.getName());
            if (!realIncomingPort.isPresent()) {
                if (rfInputPort.isPartOfPortArray()) {
                    continue;
                }

                String errorMessage = String.format(ERROR_MSG_TA_04C, realComponent.getFullName(),
                        rfInputPort.getName());
                violations.add(Violation.create(ERROR_CODE_TA_04C, errorMessage));
                if (logError) {
                    Log.error(ERROR_CODE_TA_04C.concat(errorMessage));
                }
                continue;
            }
            Port realInputPort = realIncomingPort.get();
            checkPortAgainstReferenceModel(realInputPort, rfInputPort, realComponent, violations, logError);
        }

        for (EMAPortSymbol rfOutput : rfOutputs) {
            Optional<Port> outgoingPort = realComponent.getOutgoingPort(rfOutput.getName());
            if (!outgoingPort.isPresent()) {
                if (rfOutput.isPartOfPortArray()) {
                    continue;
                }

                String errorMessage = String.format(ERROR_MSG_TA_05C, realComponent.getFullName(),
                        rfOutput.getName());
                violations.add(Violation.create(ERROR_CODE_TA_05C, errorMessage));
                if (logError) {
                    Log.error(ERROR_CODE_TA_05C.concat(errorMessage));
                }
                continue;
            }
            Port outputPort = outgoingPort.get();
            checkPortAgainstReferenceModel(outputPort, rfOutput, realComponent, violations, logError);
        }
    }

    private static void validateConnectors(Map<String, ArchitectureComponent> componentMap,
                                           EMAComponentSymbol referenceModel,
                                           boolean logError, List<Violation> violations,
                                           Set<String> componentsInComponentMap) {

        Collection<EMAConnectorSymbol> connectors = referenceModel.getConnectors();
        Set<List<String>> cartesianProduct = cartesianProduct(componentsInComponentMap);
        for (List<String> product : cartesianProduct) {
            if (product.size() != 2) {
                continue;
            }

            String first = product.get(0);
            String second = product.get(1);
            if (first.equals(second)) {
                continue;
            }

            Collection<EMAConnectorSymbol> connectorsFiltered =
                    getAllConnectorsBetween(connectors, first, second);
            for (EMAConnectorSymbol connector : connectorsFiltered) {
                EMAPortSymbol sourcePort = connector.getSourcePort();
                EMAPortSymbol targetPort = connector.getTargetPort();
                if (sourcePort == null || targetPort == null) {
                    throw new SchemaLangTechnicalException(
                            String.format("Invalid connector '%s'", connector));
                }
                if (sourcePort.isPartOfPortArray() && targetPort.isPartOfPortArray()) {
                    checkPortTypesWithConnectorArray(connector, sourcePort, targetPort, componentMap.get(first),
                            componentMap.get(second), violations, logError);
                    continue;
                }
                Optional<Port> outgoingPort = componentMap.get(first).getPort(sourcePort.getName());
                Optional<Port> incomingPort = componentMap.get(second).getPort(targetPort.getName());
                if (!outgoingPort.isPresent() || !incomingPort.isPresent()) {
                    continue;
                }
                checkPortTypesWithConnector(outgoingPort.get(), incomingPort.get(),
                        connector, violations, logError);
            }
        }
    }

    private static void checkPortTypesWithConnectorArray(EMAConnectorSymbol connector,
                                                         EMAPortSymbol sourcePort,
                                                         EMAPortSymbol targetPort,
                                                         ArchitectureComponent sourceComponent,
                                                         ArchitectureComponent targetComponent,
                                                         List<Violation> violations,
                                                         boolean logError) {

        MCTypeReference<? extends MCTypeSymbol> typeReference = sourcePort.getTypeReference();
        if (typeReference instanceof CommonMCTypeReference) {
            CommonMCTypeReference commonMCTypeReference = (CommonMCTypeReference) typeReference;
        }

        String sourcePortName = sourcePort.getNameWithoutArrayBracketPart();
        String targetPortName = targetPort.getNameWithoutArrayBracketPart();

        List<Port> outgoingPortsSource = Lists.newArrayList();
        for (Port outgoingPortSource : sourceComponent.getOutgoingPorts()){
            if (outgoingPortSource.getPortName().startsWith(sourcePortName)) {
                outgoingPortsSource.add(outgoingPortSource);
            }
        }

        List<Port> incomingPortsTarget = Lists.newArrayList();
        for (Port incomingPortTarget : targetComponent.getIncomingPorts()){
            if (incomingPortTarget.getPortName().startsWith(targetPortName)) {
                incomingPortsTarget.add(incomingPortTarget);
            }
        }

        if (outgoingPortsSource.size() != incomingPortsTarget.size()) {
            String sourcePortsJoined = outgoingPortsSource.stream().map(Port::getPortName).
                    collect(Collectors.joining(", "));
            String targetPortsJoined = incomingPortsTarget.stream().map(Port::getPortName).
                    collect(Collectors.joining(", "));
            String errorMessage = String.format(ERROR_MSG_TA_22C, connector, sourceComponent.getFullName(),
                    targetComponent.getFullName(), sourceComponent.getFullName(), outgoingPortsSource.size(),
                    targetComponent.getFullName(), incomingPortsTarget.size());
            violations.add(Violation.create(ERROR_CODE_TA_22C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_22C.concat(errorMessage));
            }

        }

        Map<Port, Port> portPortMap = correspondingPorts(outgoingPortsSource, incomingPortsTarget,
                sourceComponent, targetComponent);
        if (outgoingPortsSource.size() == incomingPortsTarget.size() && portPortMap == null) {
            String sourcePortsJoined = outgoingPortsSource.stream().map(Port::getPortName).
                    collect(Collectors.joining(", "));
            String targetPortsJoined = incomingPortsTarget.stream().map(Port::getPortName).
                    collect(Collectors.joining(", "));
            String errorMessage = String.format(ERROR_MSG_TA_23C, connector, sourceComponent.getFullName(),
                    targetComponent.getFullName(), sourceComponent.getFullName(), sourcePortsJoined,
                    targetComponent.getFullName(), targetPortsJoined);
            violations.add(Violation.create(ERROR_CODE_TA_23C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_23C.concat(errorMessage));
            }

        }
    }

    private static Map<Port, Port> correspondingPorts(List<Port> outgoingPortsSource,
                                              List<Port> incomingPortsTarget,
                                              ArchitectureComponent sourceComponent,
                                              ArchitectureComponent targetComponent) {

        Map<Port, Port> inputOutputPortMapping = Maps.newHashMap();
        for (Port outputPort : outgoingPortsSource) {
            for (Port inputPort : incomingPortsTarget) {
                String connector = outputPort.getPortName() + " -> " + inputPort.getPortName();
                if (!inputOutputPortMapping.containsKey(outputPort)
                        && inputPort.getPortName().equals(outputPort.getPortName())) {
                    inputOutputPortMapping.put(outputPort, inputPort);
                    continue;
                }
                return null;
            }
        }
        return inputOutputPortMapping;
    }

    private static void checkComponentsInConfiguration(ConfigurationSymbol configuration,
                                                       EMAComponentSymbol referenceModel,
                                                       Set<String> componentsNotInComponentMap,
                                                       List<Violation> violations,
                                                       boolean logError) {

        for (String componentName : componentsNotInComponentMap) {
            Optional<ConfigurationEntry> configurationEntryOpt = configuration.getConfigurationEntry(componentName);
            if (!configurationEntryOpt.isPresent()) {
                String errorMessage = String.format(ERROR_MSG_TA_01C, componentName);
                violations.add(Violation.create(ERROR_CODE_TA_01C, errorMessage));
                if (logError) {
                    Log.error(ERROR_CODE_TA_01C.concat(errorMessage));
                }
                continue;
            }

            Optional<EMAComponentInstantiationSymbol> subComponentOpt = referenceModel.getSubComponent(componentName);
            if (!subComponentOpt.isPresent()) {
                throw new SchemaLangTechnicalException("This should have not happen.");
            }
            EMAComponentInstantiationSymbol emaComponentInstantiationSymbol = subComponentOpt.get();
            EMAComponentSymbol component = emaComponentInstantiationSymbol.getComponentType();
            Collection<EMAPortSymbol> emaPorts = component.getPortsList();
            ConfigurationEntry configurationEntry = configurationEntryOpt.get();

            // If the configuration entry is not a nested configuration entry, then the component
            // is not allowed to have port definitions
            if (configurationEntry.isOfSymbolKind(ConfigurationEntrySymbol.KIND) && !emaPorts.isEmpty()) {
//                String ports = emaPorts.stream().map(EMAPortSymbol::getName).collect(
//                        Collectors.joining(", "));
//                String errorMessage = String.format(ERROR_MSG_TA_19C, configurationEntry.getName(), ports);
//                violations.add(Violation.create(ERROR_CODE_TA_19C, errorMessage));
//                if (logError) {
//                    Log.error(ERROR_CODE_TA_19C.concat(errorMessage));
//                }

            } else if (configurationEntry.isOfSymbolKind(NestedConfigurationEntrySymbol.KIND)) {
                Optional<NestedConfigurationEntrySymbol> nestedConfigurationEntry =
                        configuration.getConfigurationEntryOfKind(componentName, NestedConfigurationEntrySymbol.KIND);
                if (!nestedConfigurationEntry.isPresent()) {
                    throw new SchemaLangTechnicalException("This should have not happen.");
                }

                NestedConfigurationEntrySymbol nestedConfigurationEntrySymbol = nestedConfigurationEntry.get();
                List<String> notAvailablePortConfigurations = Lists.newArrayList();
                for (EMAPortSymbol emaPort : emaPorts) {
                    if (!nestedConfigurationEntrySymbol.hasConfigurationEntry(emaPort.getName())) {
                        notAvailablePortConfigurations.add(emaPort.getName());
                    }
                }

                if (!notAvailablePortConfigurations.isEmpty()) {
                    String joined = notAvailablePortConfigurations.stream().collect(Collectors.joining(", "));
                    String errorMessage = String.format(ERROR_MSG_TA_16C, joined);
                    violations.add(Violation.create(ERROR_CODE_TA_16C, errorMessage));
                    if (logError) {
                        Log.error(ERROR_CODE_TA_16C.concat(errorMessage));
                    }
                }
            }
        }
    }

    private static void checkPortTypesWithConnector(Port outputPort, Port inputPort,
                                                    EMAConnectorSymbol connector,
                                                    List<Violation> violations,
                                                    boolean logError) {

        PortType outputPortType = outputPort.getPortType();
        PortType inputPortType = inputPort.getPortType();
        final Optional<Dimension> outputPortDimensionOpt = outputPortType.getDimension();
        final Optional<Dimension> inputPortDimensionOpt = inputPortType.getDimension();

        if (inputPortDimensionOpt.isPresent() && outputPortDimensionOpt.isPresent()) {
            if (inputPortDimensionOpt.get().getSize() != outputPortDimensionOpt.get().getSize()) {
                String errorMessage = String.format(ERROR_MSG_TA_06C, connector);
                violations.add(Violation.create(ERROR_CODE_TA_06C, errorMessage));
                if (logError) {
                    Log.error(ERROR_CODE_TA_06C.concat(errorMessage));
                }
            }
        }
        checkMatrixTypeOfConnection(inputPortType, outputPortType, connector, violations, logError);
    }

    private static void checkPortAgainstReferenceModel(Port port, EMAPortSymbol rfPort,
                                                       ArchitectureComponent realComponent,
                                                       List<Violation> violations, boolean logError) {

        PortType portType = port.getPortType();
        MCTypeReference<? extends MCTypeSymbol> rfTypeReference = rfPort.getTypeReference();
        Optional<Dimension> dimensionOpt = portType.getDimension();
        Optional<Integer> referencedModelDimension = getDimension(rfTypeReference);

        if (!dimensionOpt.isPresent() && referencedModelDimension.isPresent()) {
            String errorMessage = String.format(ERROR_MSG_TA_21C, port.getPortName(),
                    realComponent.getFullName(), referencedModelDimension.get());
            violations.add(Violation.create(ERROR_CODE_TA_21C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_21C.concat(errorMessage));
            }

        } else if (dimensionOpt.isPresent() && referencedModelDimension.isPresent()
                && dimensionOpt.get().getDimensionList().size() != referencedModelDimension.get()) {
            String errorMessage = String.format(ERROR_MSG_TA_06C, port.getPortName(),
                    realComponent.getFullName(), referencedModelDimension.get(),
                    dimensionOpt.get().getDimensionList().size());
            violations.add(Violation.create(ERROR_CODE_TA_06C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_06C.concat(errorMessage));
            }
        }

        checkPortTypeAgainstReferenceModel(port.getPortName(), portType, rfTypeReference,
                realComponent, violations, logError);
    }

    private static void checkPortTypeAgainstReferenceModel(String portName, PortType realPortType,
                                                           MCTypeReference<? extends MCTypeSymbol> rmTypeReference,
                                                           ArchitectureComponent realComponent,
                                                           List<Violation> violations,
                                                           boolean logError) {

        if (rmTypeReference instanceof MCASTTypeSymbol) {
            MCASTTypeSymbolReference rmTypeSymbol = (MCASTTypeSymbolReference) rmTypeReference;
            ASTType rmAstType = rmTypeSymbol.getAstType();
            if (rmAstType instanceof ASTCommonMatrixType) {
                ASTCommonMatrixType rmMatrixType = (ASTCommonMatrixType) rmAstType;
                checkMatrixTypeAgainstReferenceModel(realPortType, rmMatrixType, portName, realComponent, violations, logError);
            }

        } else if (rmTypeReference instanceof CommonMCTypeReference) {
            CommonMCTypeReference rmCommonTypeReference = (CommonMCTypeReference) rmTypeReference;
            String rmTypeIdentifier = rmCommonTypeReference.getName();
            String realTypeIdentifier = realPortType.getTypeIdentifier();
            if (!isGenericType(rmTypeIdentifier) && !rmTypeIdentifier.equals(realTypeIdentifier)) {
                String errorMessage = String.format(ERROR_MSG_TA_20C, portName, realComponent.getFullName(),
                        rmTypeIdentifier, realTypeIdentifier);
                violations.add(Violation.create(ERROR_CODE_TA_20C, errorMessage));
                if (logError) {
                    Log.error(ERROR_CODE_TA_20C.concat(errorMessage));
                }
            }
        }
    }

    private static Optional<Integer> getDimension(MCTypeReference<? extends MCTypeSymbol> rfTypeReference) {

        if (rfTypeReference instanceof MCASTTypeSymbolReference) {
            MCASTTypeSymbolReference mcastTypeSymbolReference = (MCASTTypeSymbolReference) rfTypeReference;
            ASTCommonMatrixType astType = (ASTCommonMatrixType) mcastTypeSymbolReference.getAstType();
            ASTDimension dimension = astType.getDimension();
            return Optional.of(dimension.getDimensionList().size());
        }
        return Optional.empty();
    }

    private static void checkMatrixTypeOfConnection(PortType outputPortType, PortType inputPortType,
                                                    EMAConnectorSymbol connector,
                                                    List<Violation> violations,
                                                    boolean logError) {

        Optional<Dimension> outputDimensionOpt = outputPortType.getDimension();
        Optional<Dimension> inputDimensionOpt = inputPortType.getDimension();
        if (outputDimensionOpt.isPresent() && inputDimensionOpt.isPresent()) {
            Dimension outputDimension = outputDimensionOpt.get();
            Dimension inputDimension = inputDimensionOpt.get();
            EMAPortSymbol sourcePort = connector.getSourcePort();
            EMAComponentSymbol sourcePortComponent = sourcePort.getComponent();
            EMAPortSymbol targetPort = connector.getTargetPort();
            EMAComponentSymbol targetPortComponent = targetPort.getComponent();
            if (!inputDimension.equals(outputDimension)) {
                String errorMessage = String.format(ERROR_MSG_TA_07C, sourcePortComponent.getName(),
                        targetPortComponent.getName(), connector);
                violations.add(Violation.create(ERROR_CODE_TA_07C, errorMessage));
                if (logError) {
                    Log.error(ERROR_CODE_TA_07C.concat(errorMessage));
                }
            }
        }
        elementTypesAreCompatibleForConnector(inputPortType, outputPortType, connector, violations, logError);
    }

    private static void elementTypesAreCompatibleForConnector(PortType outputType,
                                                              PortType inputType,
                                                              EMAConnectorSymbol connector,
                                                              List<Violation> violations,
                                                              boolean logError) {

        String outputTypeName = outputType.getTypeIdentifier();
        String inputTypeName = inputType.getTypeIdentifier();
        if (!outputTypeName.equals(inputTypeName)) {
            EMAPortSymbol sourcePort = connector.getSourcePort();
            EMAComponentSymbol sourcePortComponent = sourcePort.getComponent();
            EMAPortSymbol targetPort = connector.getTargetPort();
            EMAComponentSymbol targetPortComponent = targetPort.getComponent();
            String errorMessage = String.format(ERROR_MSG_TA_08C, sourcePortComponent.getName(),
                    targetPortComponent.getName(), connector);
            violations.add(Violation.create(ERROR_CODE_TA_08C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_08C.concat(errorMessage));
            }
        }

        Optional<Range> outputRange = outputType.getRange();
        Optional<Range> inputRange = inputType.getRange();
        if (outputRange.isPresent() && !inputRange.isPresent()) {
            String errorMessage = String.format(ERROR_MSG_TA_09C, connector);
            violations.add(Violation.create(ERROR_CODE_TA_09C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_09C.concat(errorMessage));
            }
        }

        if (!outputRange.isPresent() && inputRange.isPresent()) {
            String errorMessage = String.format(ERROR_MSG_TA_10C, connector);
            violations.add(Violation.create(ERROR_CODE_TA_10C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_10C.concat(errorMessage));
            }
        }

        if (outputRange.isPresent() && inputRange.isPresent()) {
            Range rangeOutput = outputRange.get();
            Range rangeInput = inputRange.get();

            BigDecimal startValueOutput = rangeOutput.getStartValue();
            BigDecimal startValueInput = rangeInput.getStartValue();
            BigDecimal endValueOutput = rangeOutput.getEndValue();
            BigDecimal endValueInput = rangeInput.getEndValue();
            if (!((startValueOutput == null && startValueInput == null)
                    || (endValueOutput == null && endValueInput == null))) {
                if (!startValueOutput.equals(startValueInput)
                        || ! endValueOutput.equals(endValueInput)) {
                    EMAPortSymbol sourcePort = connector.getSourcePort();
                    EMAComponentSymbol sourcePortComponent = sourcePort.getComponent();
                    EMAPortSymbol targetPort = connector.getTargetPort();
                    EMAComponentSymbol targetPortComponent = targetPort.getComponent();
                    String errorMessage = String.format(ERROR_MSG_TA_11C, sourcePortComponent.getName(),
                            targetPortComponent.getName(), connector);
                    violations.add(Violation.create(ERROR_CODE_TA_11C, errorMessage));
                    if (logError) {
                        Log.error(ERROR_CODE_TA_11C.concat(errorMessage));
                    }
                }
            }
        }
    }

    private static boolean isGenericType(String typeIdentifier) {
        if (typeIdentifier.equals("B") || typeIdentifier.equals("Z") ||
                typeIdentifier.equals("Q")) {
            return false;
        }
        return true;
    }

    private static void checkMatrixTypeAgainstReferenceModel(PortType portType,
                                                             ASTCommonMatrixType rfMatrixType,
                                                             String portName, ArchitectureComponent realComponent,
                                                             List<Violation> violations,
                                                             boolean logError) {

        Optional<Dimension> dimensionOpt = portType.getDimension();
        ASTDimension rfDimension = rfMatrixType.getDimension();
        if (dimensionOpt.isPresent()) {
            Dimension dimension = dimensionOpt.get();
            matrixDimensionsAreCompatible(dimension.getDimensionList(), rfDimension.getMatrixDimList(),
                    portName, violations, logError);
        }

        ASTElementType rfElementType = rfMatrixType.getElementType();
        String typeName = portType.getTypeIdentifier();
        String rfTypeName = rfElementType.getName();
        if (!typeName.equals(rfTypeName) && !isGenericType(rfTypeName)) {
            String errorMessage = String.format(ERROR_MSG_TA_14C, portName);
            violations.add(Violation.create(ERROR_CODE_TA_14C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_14C.concat(errorMessage));
            }
        }

        Optional<Range> rangeOpt = portType.getRange();
        Optional<ASTRange> rfRange = rfElementType.getRangeOpt();
        if (!rangeOpt.isPresent() && rfRange.isPresent()) {
            ASTRange astRange = rfRange.get();
            String errorMessage = String.format(ERROR_MSG_TA_18C, portName,
                    realComponent.getFullName(), astRange);
            violations.add(Violation.create(ERROR_CODE_TA_18C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_18C.concat(errorMessage));
            }
        }

        if (rangeOpt.isPresent() && rfRange.isPresent()) {
            Range rangeOutput = rangeOpt.get();
            ASTRange rangeInput = rfRange.get();

            BigDecimal startValueOutput = rangeOutput.getStartValue();
            Rational startValueInput = rangeInput.getStartValue();
            BigDecimal endValueOutput = rangeOutput.getEndValue();
            Rational endValueInput = rangeInput.getEndValue();
            if (!startValueOutput.equals(BigDecimal.valueOf(startValueInput.doubleValue()))
                    || !endValueOutput.equals(BigDecimal.valueOf(endValueInput.doubleValue()))) {
                String errorMessage = String.format(ERROR_MSG_TA_15C, portName);
                violations.add(Violation.create(ERROR_CODE_TA_15C, errorMessage));
                if (logError) {
                    Log.error(ERROR_CODE_TA_15C.concat(errorMessage));
                }
            }
        }
    }

    private static void matrixDimensionsAreCompatible(List<Integer> matrixDimensionsOutput,
                                                      List<ASTExpression> matrixDimensionsInput,
                                                      String portName, List<Violation> violations,
                                                      boolean logError) {
        boolean isCompatible = true;
        if (matrixDimensionsOutput.isEmpty() && matrixDimensionsInput.isEmpty()) {
            return;

        } else if (matrixDimensionsOutput.size() == matrixDimensionsInput.size()) {
            for (int i = 0; i < matrixDimensionsInput.size(); i++) {
                ASTExpression astExpression = matrixDimensionsInput.get(i);
                if (astExpression instanceof ASTUnitNumberResolutionExpression) {
                    ASTUnitNumberResolutionExpression unitNumberResolutionExpression
                            = (ASTUnitNumberResolutionExpression) astExpression;
                    ASTUnitNumberResolution unitNumberResolution =
                            unitNumberResolutionExpression.getUnitNumberResolution();
                    String name = unitNumberResolution.getName();

                } else {
                    ASTUnitNumberExpression rfExpression = (ASTUnitNumberExpression) astExpression;
                    ASTNumberWithUnit rfNumberWithUnit = rfExpression.getNumberWithUnit();
                    ASTNumberWithInf numberInput = rfNumberWithUnit.getNum();
                    ASTNumericLiteral numericLiteral = numberInput.getNumber();
                    ASTIntLiteral intLiteral = (ASTIntLiteral) numericLiteral;
                    if (!matrixDimensionsOutput.get(i).equals(intLiteral.getValue())) {
                        isCompatible = false;
                        break;
                    }
                }
            }
        }

        if (!isCompatible) {
            String errorMessage = String.format(ERROR_MSG_TA_13C, portName);
            violations.add(Violation.create(ERROR_CODE_TA_13C, errorMessage));
            if (logError) {
                Log.error(ERROR_CODE_TA_13C.concat(errorMessage));
            }
        }
    }

    private static Set<List<String>> cartesianProduct(Set<String> set) {

        Set<List<String>> result = Sets.newHashSet();
        Iterator<String> iterator = set.iterator();
        if (!iterator.hasNext()) {
            return result;
        }

        String first = iterator.next();
        while (iterator.hasNext()) {
            String next = iterator.next();
            result.add(Lists.newArrayList(first, next));
            result.add(Lists.newArrayList(next, first));
        }
        return result;
    }

    private static Collection<EMAConnectorSymbol> getAllConnectorsBetween(Collection<EMAConnectorSymbol> connectors, String source, String target) {

        Collection<EMAConnectorSymbol> connectorsFiltered = Lists.newArrayList();
        for (EMAConnectorSymbol connector : connectors) {
            Optional<ASTNode> astConnectorOpt = connector.getAstNode();
            if (!astConnectorOpt.isPresent()) {
                throw new SchemaLangTechnicalException("This should have not happen.");
            }
            ASTConnector astConnector = (ASTConnector) astConnectorOpt.get();
            String compNameSource = astConnector.getSource().getQualifiedNameWithArray().getCompName();
            String compNameTarget = astConnector.getTargets().getQualifiedNameWithArrayAndStarList().get(0).getQualifiedNameWithArray().getCompName();

            if (compNameSource.equals(source) && compNameTarget.equals(target)) {
                connectorsFiltered.add(connector);
            }
        }
        return connectorsFiltered;
    }
}