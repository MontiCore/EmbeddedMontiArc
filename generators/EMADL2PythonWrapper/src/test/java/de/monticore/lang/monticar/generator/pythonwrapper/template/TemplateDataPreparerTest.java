/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppVariableViewModel;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppWrapperViewModel;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static org.assertj.core.api.AssertionsForClassTypes.assertThat;
import static org.assertj.core.api.AssertionsForClassTypes.assertThatExceptionOfType;


/**
 *
 */
public class TemplateDataPreparerTest {
    private static final String ANY_VARIABLE_NAME = "anyName";
    private static final PortDirection ANY_PORT_DIRECTION = PortDirection.INPUT;
    private static final List<Integer> ANY_VECTOR_DIMENSION = Lists.newArrayList(3);
    private static final List<Integer> ANY_MATRIX_DIMENSION = Lists.newArrayList(2, 4);
    private static final List<Integer> ANY_CUBE_DIMENSION = Lists.newArrayList(4, 5, 6);

    private static final String ANY_COMPONENT_NAME = "any_component_name";

    private TemplateDataPreparer templateDataPreparer;

    @Before
    public void setup() {
        this.templateDataPreparer = new TemplateDataPreparer();
        Log.enableFailQuick(false);
    }

    @Test
    public void whenEmadlComponentInformationIsGivenShouldReturnCorrectViewInformation() {
        // given
        PortVariable inputVar1 = PortVariable.primitiveVariableFrom("inputVar1", EmadlType.Z, PortDirection.INPUT);
        PortVariable inputVar2 = PortVariable.multidimensionalVariableFrom("inputVar2", EmadlType.Q, PortDirection.INPUT, ANY_MATRIX_DIMENSION);
        PortVariable inputVar3 = PortVariable.multidimensionalVariableFrom("inputVar3", EmadlType.Z, PortDirection.INPUT, ANY_CUBE_DIMENSION);

        PortVariable outputVar1 = PortVariable.primitiveVariableFrom("outputVar1", EmadlType.Q, PortDirection.OUTPUT);
        PortVariable outputVar2 = PortVariable.multidimensionalVariableFrom("outputVar2", EmadlType.Z, PortDirection.OUTPUT, ANY_VECTOR_DIMENSION);

        List<CppVariableViewModel> expectedInputVariables = Lists.newArrayList(
                new CppVariableViewModel("int", "inputVar1"),
                new CppVariableViewModel("arma::mat", "inputVar2"),
                new CppVariableViewModel("arma::icube", "inputVar3")
        );
        List<CppVariableViewModel> expectedOutputVariables = Lists.newArrayList(
                new CppVariableViewModel("double", "outputVar1"),
                new CppVariableViewModel("arma::ivec", "outputVar2")
        );

        ComponentPortInformation componentPortInformation = new ComponentPortInformation(ANY_COMPONENT_NAME);
        componentPortInformation.addInput(inputVar1);
        componentPortInformation.addInput(inputVar2);
        componentPortInformation.addInput(inputVar3);
        componentPortInformation.addOutput(outputVar1);
        componentPortInformation.addOutput(outputVar2);

        // when
        CppWrapperViewModel result = this.templateDataPreparer.getWrapperViewModel(componentPortInformation);

        // then
        assertThat(result.getInputClassName()).isEqualTo("any_component_name_input");
        assertThat(result.getOutputClassName()).isEqualTo("any_component_name_output");
        assertThat(result.getWrappedComponentName()).isEqualTo("any_component_name");
        assertThat(result.getWrapperClassName()).isEqualTo("any_component_name_executor");
        assertThat(result.getCapitalizedWrapperClassName()).isEqualTo("ANY_COMPONENT_NAME_EXECUTOR");
        assertThat(result.getInputVariables()).isEqualTo(expectedInputVariables);
        assertThat(result.getOutputVariables()).isEqualTo(expectedOutputVariables);
    }

    @Test
    public void whenPrimitiveZEmadlTypeIsGivenReturnIntViewModel() {
        // given
        PortVariable primitiveZType = PortVariable.primitiveVariableFrom(ANY_VARIABLE_NAME, EmadlType.Z,
                ANY_PORT_DIRECTION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(primitiveZType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("int");
    }

    @Test
    public void whenPrimitiveQEmadlTypeIsGivenReturnDoubleViewModel() {
        // given
        PortVariable primitiveQType = PortVariable.primitiveVariableFrom(ANY_VARIABLE_NAME, EmadlType.Q,
                ANY_PORT_DIRECTION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(primitiveQType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("double");
    }

    @Test
    public void whenPrimitiveBEmadlTypeIsGivenReturnBoolViewModel() {
        // given
        PortVariable primitiveBType = PortVariable.primitiveVariableFrom(ANY_VARIABLE_NAME, EmadlType.B,
                ANY_PORT_DIRECTION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(primitiveBType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("bool");
    }

    @Test
    public void whenVectorZEmadlTypeIsGivenReturnIVecViewModel() {
        // given
        PortVariable vecZType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.Z,
                ANY_PORT_DIRECTION, ANY_VECTOR_DIMENSION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(vecZType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("arma::ivec");
    }

    @Test
    public void whenVectorQEmadlTypeIsGivenReturnIVecViewModel() {
        // given
        PortVariable vecQType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.Q,
                ANY_PORT_DIRECTION, ANY_VECTOR_DIMENSION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(vecQType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("arma::colvec");
    }

    @Test
    public void whenVectorBEmadlTypeIsGivenThrowViewCreationException() {
        // given
        PortVariable vecBType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.B,
                ANY_PORT_DIRECTION, ANY_VECTOR_DIMENSION);

        // when
        assertThatExceptionOfType(ViewCreationException.class)
                .isThrownBy(() -> templateDataPreparer.getVariableViewModel(vecBType));
    }

    @Test
    public void whenMatrixZEmadlTypeIsGivenReturnIMatViewModel() {
        // given
        PortVariable matZType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.Z,
                ANY_PORT_DIRECTION, ANY_MATRIX_DIMENSION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(matZType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("arma::imat");
    }

    @Test
    public void whenMatrixQEmadlTypeIsGivenReturnMatViewModel() {
        // given
        PortVariable matQType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.Q,
                ANY_PORT_DIRECTION, ANY_MATRIX_DIMENSION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(matQType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("arma::mat");
    }

    @Test
    public void whenMatrixBEmadlTypeIsGivenThrowViewCreationException() {
        // given
        PortVariable matBType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.B,
                ANY_PORT_DIRECTION, ANY_MATRIX_DIMENSION);

        // when
        assertThatExceptionOfType(ViewCreationException.class)
                .isThrownBy(() -> templateDataPreparer.getVariableViewModel(matBType));
    }

    @Test
    public void whenCubeZEmadlTypeIsGivenReturnICubeViewModel() {
        // given
        PortVariable cubeZType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.Z,
                ANY_PORT_DIRECTION, ANY_CUBE_DIMENSION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(cubeZType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("arma::icube");
    }

    @Test
    public void whenCubeQEmadlTypeIsGivenReturnCubeViewModel() {
        // given
        PortVariable cubeQType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.Q,
                ANY_PORT_DIRECTION, ANY_CUBE_DIMENSION);

        // when
        CppVariableViewModel result = templateDataPreparer.getVariableViewModel(cubeQType);

        // then
        assertThat(result.getVariableName()).isEqualTo(ANY_VARIABLE_NAME);
        assertThat(result.getType()).isEqualTo("arma::cube");
    }

    @Test
    public void whenCubeBEmadlTypeIsGivenThrowViewCreationException() {
        // given
        PortVariable cubeBType = PortVariable.multidimensionalVariableFrom(ANY_VARIABLE_NAME, EmadlType.B,
                ANY_PORT_DIRECTION, ANY_VECTOR_DIMENSION);

        // when
        assertThatExceptionOfType(ViewCreationException.class)
                .isThrownBy(() -> templateDataPreparer.getVariableViewModel(cubeBType));
    }
}
