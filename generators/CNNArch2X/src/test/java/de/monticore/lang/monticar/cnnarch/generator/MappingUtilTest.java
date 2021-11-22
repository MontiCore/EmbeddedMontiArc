package de.monticore.lang.monticar.cnnarch.generator;

public class MappingUtilTest {

//    @Test
//    public void componentFullNameIsMapped() {
//
//        /* Arrange */
//        ArchitectureAdapter trainedArchitecture = NNArchitectureMockFactory.createNNArchitectureMock();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        assertNotNull(architectureComponent);
//        assertThat(architectureComponent.getFullName(), equalTo("FullNameIsMapped"));
//    }

//    @Test
//    public void portsAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortsAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        List<Port> ports = architectureComponent.getPorts();
//        assertNotNull(ports);
//        assertThat(ports.size(), equalTo(2));
//    }
//
//    @Test
//    public void portNamesAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortNamesAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        List<Port> ports = architectureComponent.getPorts();
//        assertNotNull(ports);
//        assertThat(ports.size(), equalTo(4));
//        assertTrue(architectureComponent.getIncomingPort("input_port").isPresent());
//        assertTrue(architectureComponent.getIncomingPort("input_port2").isPresent());
//        assertTrue(architectureComponent.getOutgoingPort("output_port").isPresent());
//        assertTrue(architectureComponent.getOutgoingPort("output_port2").isPresent());
//    }
//
//    @Test
//    public void portTypesAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortTypesAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        Optional<Port> inputPortOpt = architectureComponent.getPort("input_port");
//        assertTrue(inputPortOpt.isPresent());
//        Port inputPort = inputPortOpt.get();
//        PortType inputPortType = inputPort.getPortType();
//        assertNotNull(inputPortType);
//
//        Optional<Port> outputPortOpt = architectureComponent.getPort("output_port");
//        assertTrue(outputPortOpt.isPresent());
//        Port outputPort = outputPortOpt.get();
//        PortType outputPortType = outputPort.getPortType();
//        assertNotNull(outputPortType);
//    }
//
//    @Test
//    public void portTypeIdentifiersAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortTypeIdentifiersAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        Optional<Port> inputPortOpt = architectureComponent.getPort("input_port");
//        assertTrue(inputPortOpt.isPresent());
//        Port inputPort = inputPortOpt.get();
//        PortType inputPortType = inputPort.getPortType();
//        assertNotNull(inputPortType);
//        assertThat(inputPortType.getTypeIdentifier(), equalTo("Q"));
//
//        Optional<Port> outputPortOpt = architectureComponent.getPort("output_port");
//        assertTrue(outputPortOpt.isPresent());
//        Port outputPort = outputPortOpt.get();
//        PortType outputPortType = outputPort.getPortType();
//        assertNotNull(outputPortType);
//        assertThat(outputPortType.getTypeIdentifier(), equalTo("Z"));
//    }
//
//    @Test
//    public void portRangesAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortRangesAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        Optional<Port> inputPortOpt = architectureComponent.getPort("input_port");
//        assertTrue(inputPortOpt.isPresent());
//        Port inputPort = inputPortOpt.get();
//        PortType inputPortType = inputPort.getPortType();
//        assertNotNull(inputPortType);
//        Optional<Range> inputPortRangeOpt = inputPortType.getRange();
//        assertTrue(inputPortRangeOpt.isPresent());
//
//        Optional<Port> outputPortOpt = architectureComponent.getPort("output_port");
//        assertTrue(outputPortOpt.isPresent());
//        Port outputPort = outputPortOpt.get();
//        PortType outputPortType = outputPort.getPortType();
//        assertNotNull(outputPortType);
//        Optional<Range> outputPortRangeOpt = outputPortType.getRange();
//        assertTrue(outputPortRangeOpt.isPresent());
//    }
//
//    @Test
//    public void portRangeBoundariesAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortRangeBoundariesAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        Optional<Port> inputPortOpt = architectureComponent.getPort("input_port");
//        assertTrue(inputPortOpt.isPresent());
//        Port inputPort = inputPortOpt.get();
//        PortType inputPortType = inputPort.getPortType();
//        assertNotNull(inputPortType);
//        Optional<Range> inputPortRangeOpt = inputPortType.getRange();
//        assertTrue(inputPortRangeOpt.isPresent());
//        Range inputPortRange = inputPortRangeOpt.get();
//        assertThat(inputPortRange.getStartValue(), equalTo(BigDecimal.valueOf(-0.5)));
//        assertThat(inputPortRange.getEndValue(), equalTo(BigDecimal.valueOf(0.5)));
//
//        Optional<Port> outputPortOpt = architectureComponent.getPort("output_port");
//        assertTrue(outputPortOpt.isPresent());
//        Port outputPort = outputPortOpt.get();
//        PortType outputPortType = outputPort.getPortType();
//        assertNotNull(outputPortType);
//        Optional<Range> outputPortRangeOpt = outputPortType.getRange();
//        assertTrue(outputPortRangeOpt.isPresent());
//        Range outputPortRange = outputPortRangeOpt.get();
//        assertThat(outputPortRange.getStartValue(), equalTo(BigDecimal.valueOf(-1.0)));
//        assertThat(outputPortRange.getEndValue(), equalTo(BigDecimal.valueOf(1.0)));
//    }
//
//    @Test
//    public void portDimensionsAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortDimensionsAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        Optional<Port> inputPortOpt = architectureComponent.getPort("input_port");
//        assertTrue(inputPortOpt.isPresent());
//        Port inputPort = inputPortOpt.get();
//        PortType inputPortType = inputPort.getPortType();
//        assertNotNull(inputPortType);
//        Optional<Dimension> inputPortDimensionOpt = inputPortType.getDimension();
//        assertTrue(inputPortDimensionOpt.isPresent());
//
//        Optional<Port> outputPortOpt = architectureComponent.getPort("output_port");
//        assertTrue(outputPortOpt.isPresent());
//        Port outputPort = outputPortOpt.get();
//        PortType outputPortType = outputPort.getPortType();
//        assertNotNull(outputPortType);
//        Optional<Dimension> outputPortDimensionOpt = outputPortType.getDimension();
//        assertTrue(outputPortDimensionOpt.isPresent());
//    }
//
//    @Test
//    public void portDimensionNumbersAreMapped() {
//
//        /* Arrange */
//        ASTComponent component = parseEMAComponent("src/test/resources/ema/mapping/" +
//                "PortDimensionNumbersAreMapped.ema");
//        createEMASymbolTable(component);
//        EMAComponentSymbol componentSymbol = (EMAComponentSymbol) component.getSymbol();
//
//        /* Act */
//        ArchitectureComponent architectureComponent = MappingUtils.createArchitectureComponent(componentSymbol);
//
//        /* Assert */
//        Optional<Port> inputPortOpt = architectureComponent.getPort("input_port");
//        assertTrue(inputPortOpt.isPresent());
//        Port inputPort = inputPortOpt.get();
//        PortType inputPortType = inputPort.getPortType();
//        assertNotNull(inputPortType);
//        Optional<Dimension> inputPortDimensionOpt = inputPortType.getDimension();
//        assertTrue(inputPortDimensionOpt.isPresent());
//        Dimension inputPortDimension = inputPortDimensionOpt.get();
//        assertThat(inputPortDimension.getDimensionList(), hasItems(2, 2));
//
//        Optional<Port> outputPortOpt = architectureComponent.getPort("output_port");
//        assertTrue(outputPortOpt.isPresent());
//        Port outputPort = outputPortOpt.get();
//        PortType outputPortType = outputPort.getPortType();
//        assertNotNull(outputPortType);
//        Optional<Dimension> outputPortDimensionOpt = outputPortType.getDimension();
//        assertTrue(outputPortDimensionOpt.isPresent());
//        Dimension outputPortDimension = outputPortDimensionOpt.get();
//        assertThat(outputPortDimension.getDimensionList(), hasItems(1, 1));
//    }
}