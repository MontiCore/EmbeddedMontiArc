/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.*;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.ComponentInstanceSymbolLibraryFactory;

import java.util.List;

import static org.assertj.core.api.Java6Assertions.assertThat;

/**
 *
 */
public class EmadlComponentTranslatorIntegrationTest {
    private EmadlComponentTranslator translator;

    private final ComponentInstanceSymbolLibraryFactory componentLibrary = new ComponentInstanceSymbolLibraryFactory();

    @Before
    public void setup() {
        this.translator = new EmadlComponentTranslatorFactory().create();
    }

    @Test
    public void integrationTestForAllTypesModel() {
        // given
        EMAComponentInstanceSymbol allTypesComponent = componentLibrary
                .getComponentInstanceLibrary()
                .getModelByIdentifier(ComponentInstanceSymbolLibraryFactory.ALL_TYPES_MODEL);

        // when
        ComponentPortInformation result = translator.extractPortInformationFrom(allTypesComponent);

        // then
        List<PortVariable> inputs = result.getAllInputs();
        List<PortVariable> outputs = result.getAllOutputs();
        assertThat(result.getComponentName()).isEqualTo("types_allTypes");
        assertThat(inputs).containsExactly(
                PortVariable.primitiveVariableFrom("q1", EmadlType.Q, PortDirection.INPUT),
                PortVariable.multidimensionalVariableFrom("q2", EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(4)),
                PortVariable.multidimensionalVariableFrom("q3", EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(3, 6)),
                PortVariable.multidimensionalVariableFrom("q4", EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(2, 3, 5)),
                PortVariable.primitiveVariableFrom("z1", EmadlType.Z, PortDirection.INPUT),
                PortVariable.multidimensionalVariableFrom("z2", EmadlType.Z, PortDirection.INPUT, Lists.newArrayList(6)),
                PortVariable.multidimensionalVariableFrom("z3", EmadlType.Z, PortDirection.INPUT, Lists.newArrayList(7, 3)),
                PortVariable.multidimensionalVariableFrom("z4", EmadlType.Z, PortDirection.INPUT, Lists.newArrayList(2, 2, 7)),
                PortVariable.multidimensionalVariableFrom("z5", EmadlType.Z, PortDirection.INPUT, Lists.newArrayList(1)),
                PortVariable.primitiveVariableFrom("b1", EmadlType.B, PortDirection.INPUT)
        );
        assertThat(outputs).containsExactly(
                PortVariable.primitiveVariableFrom("result", EmadlType.B, PortDirection.OUTPUT)
        );
    }

    @Test
    public void integrationTestForIoTestModel() {
        // given
        EMAComponentInstanceSymbol ioTestComponent = componentLibrary
                .getComponentInstanceLibrary()
                .getModelByIdentifier(ComponentInstanceSymbolLibraryFactory.IO_TEST_MODEL);

        //when
        ComponentPortInformation result = translator.extractPortInformationFrom(ioTestComponent);

        // then
        List<PortVariable> inputs = result.getAllInputs();
        List<PortVariable> outputs = result.getAllOutputs();
        assertThat(result.getComponentName()).isEqualTo("io_compTestIO");
        assertThat(inputs).containsExactly(
                PortVariable.primitiveVariableFrom("inp1", EmadlType.Q, PortDirection.INPUT),
                PortVariable.primitiveVariableFrom("inp2", EmadlType.Z, PortDirection.INPUT),
                PortVariable.primitiveVariableFrom("inp3", EmadlType.B, PortDirection.INPUT)
        );
        assertThat(outputs).containsExactly(
                PortVariable.multidimensionalVariableFrom("out1", EmadlType.Z, PortDirection.OUTPUT, Lists.newArrayList(3)),
                PortVariable.multidimensionalVariableFrom("out2", EmadlType.Q, PortDirection.OUTPUT, Lists.newArrayList(3, 4))
        );
    }

    @Test
    public void integrationTestForCalculatorModel() {
        // given
        EMAComponentInstanceSymbol calculatorModel = componentLibrary
                .getComponentInstanceLibrary()
                .getModelByIdentifier(ComponentInstanceSymbolLibraryFactory.CALCULATOR_MODEL);

        //when
        ComponentPortInformation result = translator.extractPortInformationFrom(calculatorModel);

        // then
        List<PortVariable> inputs = result.getAllInputs();
        List<PortVariable> outputs = result.getAllOutputs();
        assertThat(result.getComponentName()).isEqualTo("calculator_calculator");
        assertThat(inputs).containsExactly(
                PortVariable.multidimensionalVariableFrom("a", EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(3)),
                PortVariable.multidimensionalVariableFrom("b", EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(3)),
                PortVariable.primitiveVariableFrom("c", EmadlType.Z, PortDirection.INPUT)
        );
        assertThat(outputs).containsExactly(
                PortVariable.primitiveVariableFrom("result", EmadlType.Q, PortDirection.OUTPUT)
        );
    }
}
