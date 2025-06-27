/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.ComponentInstanceSymbolLibraryFactory;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.*;

/**
 *
 */
public class EmadlComponentTranslatorTest {
    private EmadlComponentTranslator translator;

    private EmadlInstanceSymbol2PortVariableMapper portMapper;

    private final static PortVariable INPUT_PORT_VALUE_1 = PortVariable.primitiveVariableFrom("inpval1",
            EmadlType.Z, PortDirection.INPUT);
    private final static PortVariable INPUT_PORT_VALUE_2 = PortVariable.primitiveVariableFrom("inpval2",
            EmadlType.Q, PortDirection.INPUT);
    private final static PortVariable INPUT_PORT_VALUE_3 = PortVariable.multidimensionalVariableFrom("inpval3",
            EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(2, 2, 3));
    private final static PortVariable OUTPUT_PORT_VALUE_1 = PortVariable.primitiveVariableFrom("outval1",
            EmadlType.B, PortDirection.OUTPUT);
    private final static PortVariable OUTPUT_PORT_VALUE_2 = PortVariable.multidimensionalVariableFrom("outval2",
            EmadlType.Z, PortDirection.OUTPUT, Lists.newArrayList(3, 4));

    @Before
    public void setup() {
        portMapper = mock(EmadlInstanceSymbol2PortVariableMapper.class, RETURNS_DEEP_STUBS);
        translator = new EmadlComponentTranslator(portMapper);

        when(portMapper.willAccept(any())).thenReturn(true);
    }

    @Test
    public void whenComponentInstanceIsGivenReturnCppComponentWithCorrectNameAndVariableList() {
        // given
        EMAComponentInstanceSymbol componentSymbol = (new ComponentInstanceSymbolLibraryFactory())
                .getComponentInstanceLibrary()
                .getModelByIdentifier("io-test");
        when(portMapper.map(any())).thenReturn(INPUT_PORT_VALUE_1, INPUT_PORT_VALUE_2,
                INPUT_PORT_VALUE_3, OUTPUT_PORT_VALUE_1, OUTPUT_PORT_VALUE_2);

        // when
        ComponentPortInformation result = translator.extractPortInformationFrom(componentSymbol);

        // then
        verify(portMapper, atLeast(5)).map(any());
        assertThat(result.getComponentName()).isEqualTo("io_compTestIO");
        assertThat(result.getAllOutputs()).containsExactly(OUTPUT_PORT_VALUE_1, OUTPUT_PORT_VALUE_2);
        assertThat(result.getAllInputs()).containsExactly(INPUT_PORT_VALUE_1, INPUT_PORT_VALUE_2, INPUT_PORT_VALUE_3);
    }

    @Test
    public void whenComponentInstanceSymbolNameIsEmptyThenWillNotAccept() {
        // given
        EMAComponentInstanceSymbol componentSymbol = (new ComponentInstanceSymbolLibraryFactory())
                .getComponentInstanceLibrary()
                .getModelByIdentifier("io-test");
        componentSymbol.setFullName("");

        // when
        boolean result = translator.willAccept(componentSymbol);

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenPortMapperWillNotAcceptThenWillNotAccept() {
        // given
        EMAComponentInstanceSymbol componentSymbol = (new ComponentInstanceSymbolLibraryFactory())
                .getComponentInstanceLibrary()
                .getModelByIdentifier("io-test");
        when(portMapper.map(any())).thenReturn(INPUT_PORT_VALUE_1, INPUT_PORT_VALUE_2,
                INPUT_PORT_VALUE_3, OUTPUT_PORT_VALUE_1, OUTPUT_PORT_VALUE_2);
        when(portMapper.willAccept(any())).thenReturn(false);


        // when
        boolean result = translator.willAccept(componentSymbol);

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenCorrectComponentSymbolThenWillAccept() {
        // given
        EMAComponentInstanceSymbol componentSymbol = (new ComponentInstanceSymbolLibraryFactory())
                .getComponentInstanceLibrary()
                .getModelByIdentifier("io-test");
        when(portMapper.map(any())).thenReturn(INPUT_PORT_VALUE_1, INPUT_PORT_VALUE_2,
                INPUT_PORT_VALUE_3, OUTPUT_PORT_VALUE_1, OUTPUT_PORT_VALUE_2);

        // when
        boolean result = translator.willAccept(componentSymbol);

        // then
        assertThat(result).isTrue();


    }
}
