package de.monticore.lang.monticar.cnnarch.generator.training;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.generator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnnarch.generator.annotations.Range;
import schemalang.validation.model.*;

import java.math.BigDecimal;
import java.util.List;
import java.util.Map;

public class MappingUtils {

    public static ArchitectureComponent createArchitectureComponent(ArchitectureAdapter architectureAdapter) {
        ArchitectureComponent architectureComponent = new ArchitectureComponent();
        architectureComponent.setFullName(architectureAdapter.getArchitectureSymbol().getComponentName());
        architectureComponent.setPorts(mapPorts(architectureAdapter.getInputs(), architectureAdapter.getOutputs(),
                architectureAdapter.getTypes(), architectureAdapter.getRanges(), architectureAdapter.getDimensions()));
        return architectureComponent;
    }

    private static List<Port> mapPorts(List<String> inputPorts, List<String> outputPorts, Map<String, String> types,
                                       Map<String, Range> ranges, Map<String, List<Integer>> dimensions) {
        List<Port> ports = Lists.newArrayList();
        for (String inputPort : inputPorts) {
            ports.add(mapPort(true, inputPort, types.get(inputPort), ranges.get(inputPort), dimensions.get(inputPort)));
        }
        for (String outputPort : outputPorts) {
            ports.add(mapPort(false, outputPort, types.get(outputPort), ranges.get(outputPort), dimensions.get(outputPort)));
        }
        return ports;
    }

    private static Port mapPort(boolean isInput, String portName, String type, Range range, List<Integer> dimensionList) {
        Port port = new Port();
        port.setPortName(portName);
        if (isInput) {
            port.setPortDirection(PortDirection.INPUT);
        } else {
            port.setPortDirection(PortDirection.OUTPUT);
        }
        port.setPortType(mapPortType(type, range, dimensionList));
        return port;
    }

    private static PortType mapPortType(String type, Range range, List<Integer> dimensionList) {
        PortType portType = new PortType();
        portType.setTypeIdentifier(type);
        portType.setRange(mapRange(range));
        portType.setDimension(mapDimension(dimensionList));
        return portType;
    }

    private static schemalang.validation.model.Range mapRange(Range range) {
        schemalang.validation.model.Range ran = new schemalang.validation.model.Range();
        if (range.getLowerLimit().isPresent() && range.getUpperLimit().isPresent()) {
            ran.setStartValue(BigDecimal.valueOf(range.getLowerLimit().get()));
            ran.setEndValue(BigDecimal.valueOf(range.getUpperLimit().get()));
        }
        return ran;
    }

    private static Dimension mapDimension(List<Integer> dimensionList) {
        Dimension dimension = new Dimension();
        dimension.setDimensionList(dimensionList);
        return dimension;
    }
}