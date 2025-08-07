/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.ComponentPortInformationBuilder;
import de.monticore.lang.monticar.generator.pythonwrapper.util.GeneratedWrapperFileSet;

import java.io.IOException;
import java.util.Map;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;
import static org.assertj.core.api.Assumptions.assumeThat;

/**
 *
 */
public class TemplateIntegrationTest {
    private TemplateController templateController;

    private static GeneratedWrapperFileSet EXPECTED_CALCULATOR_FILES = null;
    private static GeneratedWrapperFileSet EXPECTED_ALL_TYPES_FILES = null;

    static {
        try {
            EXPECTED_CALCULATOR_FILES = GeneratedWrapperFileSet.fromNameAndUrl("calculator_calculator",
            TemplateIntegrationTest.class.getResource("/calculator/calculator/pythonwrapper"));

            EXPECTED_ALL_TYPES_FILES = GeneratedWrapperFileSet.fromNameAndUrl("types_allTypes",
                    TemplateIntegrationTest.class.getResource("/all-types-model/types/pythonwrapper"));
        } catch (IOException e) {
            fail("Cannot load test files: " + e.getMessage());
        }
    }

    @Before
    public void setup() {
        this.templateController = new PythonWrapperTemplateControllerFactory().create();
    }

    @Test
    public void calculatorTestCase() {
        assumeThat(EXPECTED_CALCULATOR_FILES).isNotNull();

        // given
        ComponentPortInformation componentPortInformation = (new ComponentPortInformationBuilder())
                .withName("calculator_calculator")
                .withInputVariable(
                        PortVariable.multidimensionalVariableFrom("a", EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(3)),
                        PortVariable.multidimensionalVariableFrom("b", EmadlType.Q, PortDirection.INPUT, Lists.newArrayList(3)),
                        PortVariable.primitiveVariableFrom("c", EmadlType.Z, PortDirection.INPUT)
                )
                .withOutputVariable(
                        PortVariable.primitiveVariableFrom("result", EmadlType.Q, PortDirection.OUTPUT)
                )
                .build();

        // when
        Map<String, String> generatedFiles = this.templateController.processWrapper(componentPortInformation);

        // then
        assertThat(generatedFiles).hasSize(4);
        assertThat(generatedFiles).containsKeys(
                "calculator_calculator_executor.h",
                "calculator_calculator_executor.cpp",
                "calculator_calculator_executor.i",
                "CMakeLists.txt"
        );
        assertThat(generatedFiles.get("calculator_calculator_executor.h")).isEqualTo(EXPECTED_CALCULATOR_FILES.getWrapperHeader());
        assertThat(generatedFiles.get("calculator_calculator_executor.cpp")).isEqualTo(EXPECTED_CALCULATOR_FILES.getWrapperCpp());
        assertThat(generatedFiles.get("calculator_calculator_executor.i")).isEqualTo(EXPECTED_CALCULATOR_FILES.getSwigInterface());
        assertThat(generatedFiles.get("CMakeLists.txt")).isEqualTo(EXPECTED_CALCULATOR_FILES.getCmake());
    }

    @Test
    public void allTypesTestCase() {
        assumeThat(EXPECTED_ALL_TYPES_FILES).isNotNull();

        // given
        ComponentPortInformation componentPortInformation = (new ComponentPortInformationBuilder())
                .withName("types_allTypes")
                .withInputVariable(
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
                )
                .withOutputVariable(
                        PortVariable.primitiveVariableFrom("result", EmadlType.Z, PortDirection.OUTPUT)
                )
                .build();

        // when
        Map<String, String> generatedFiles = this.templateController.processWrapper(componentPortInformation);

        // then
        assertThat(generatedFiles).hasSize(4);
        assertThat(generatedFiles).containsKeys(
                "types_allTypes_executor.h",
                "types_allTypes_executor.cpp",
                "types_allTypes_executor.i",
                "CMakeLists.txt"
        );
        assertThat(generatedFiles.get("types_allTypes_executor.h")).isEqualTo(EXPECTED_ALL_TYPES_FILES.getWrapperHeader());
        assertThat(generatedFiles.get("types_allTypes_executor.cpp")).isEqualTo(EXPECTED_ALL_TYPES_FILES.getWrapperCpp());
        assertThat(generatedFiles.get("types_allTypes_executor.i")).isEqualTo(EXPECTED_ALL_TYPES_FILES.getSwigInterface());
        assertThat(generatedFiles.get("CMakeLists.txt")).isEqualTo(EXPECTED_ALL_TYPES_FILES.getCmake());
    }
}
