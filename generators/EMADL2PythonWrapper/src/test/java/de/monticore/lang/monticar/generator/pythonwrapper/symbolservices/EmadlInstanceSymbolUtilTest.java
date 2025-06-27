/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.SymbolMockBuilder;
import de.monticore.lang.monticar.generator.pythonwrapper.util.SymbolTestCase;
import de.monticore.lang.monticar.generator.pythonwrapper.util.SymbolTestCaseLibrary;
import de.monticore.lang.monticar.generator.pythonwrapper.util.SymbolTestCaseLibraryFactory;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;
import static org.mockito.Mockito.*;

/**
 *
 */
public class EmadlInstanceSymbolUtilTest {
    private EmadlInstanceSymbolUtil symbolUtil;
    private SymbolTestCaseLibrary testCaseLibrary;

    @Before
    public void setup() {
        symbolUtil = new EmadlInstanceSymbolUtil();

        SymbolTestCaseLibraryFactory testCaseLibraryFactory = new SymbolTestCaseLibraryFactory();
        testCaseLibrary = testCaseLibraryFactory.getSymbolTestCaseLibrary();
    }


    /************************* retrieveSymbolName Tests *************************/

    @Test
    public void whenSymbolIsGivenShouldReturnSymbolName() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;

        // when
        String result = symbolUtil.retrieveSymbolName(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(testCase.getActualSymbolName());
    }

    @Test
    public void whenNullIsGivenThenshouldReturnNullPointerException() {
        assertThatExceptionOfType(NullPointerException.class)
                .isThrownBy(() -> symbolUtil.retrieveSymbolName(null));
    }


    /************************* retrievePortType Tests *************************/


    @Test
    public void whenIncomingSymbolIsGivenThenReturnPortTypeInput() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;

        // when
        PortDirection result = symbolUtil.retrievePortType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(PortDirection.INPUT);
    }

    @Test
    public void whenIncomingSymbolIsGivenThenReturnPortTypeOutput() {
        // given
        SymbolTestCase testCase = testCaseLibrary.OUTPUT_PRIMITIVE_Z_SYMBOL;

        // when
        PortDirection result = symbolUtil.retrievePortType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(PortDirection.OUTPUT);
    }

    @Test
    public void whenIncomingSymbolIsNullThenThrowNullpointer() {
        assertThatExceptionOfType(NullPointerException.class)
                .isThrownBy(() -> symbolUtil.retrievePortType(null));
    }


    /************************* isCommonMatrixType Tests *************************/

    @Test
    public void whenSymbolIsCommonMatrixTypeThenReturnTrue() {
        // given
        SymbolTestCase testCase = testCaseLibrary.MATRIX_Z_SYMBOL;

        // when
        boolean result = symbolUtil.isCommonMatrixType(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsNotACommonMatrixTypeThenReturnTrue() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;

        // when
        boolean result = symbolUtil.isCommonMatrixType(testCase.getSymbol());

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenSymbolForCommonMatrixTypeIsNullThenThrowNullpointerException() {
        assertThatExceptionOfType(NullPointerException.class)
                .isThrownBy(() -> symbolUtil.isCommonMatrixType(null));
    }

    /************************* isPrimitiveMatrixType Tests *************************/
    @Test
    public void whenSymbolIsPrimitiveQTypeThenReturnTrue() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Q_SYMBOL;

        // when
        boolean result = symbolUtil.isPrimitiveType(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsPrimitiveZTypeThenReturnTrue() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;

        // when
        boolean result = symbolUtil.isPrimitiveType(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsPrimitiveBTypeThenReturnTrue() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_B_SYMBOL;

        // when
        boolean result = symbolUtil.isPrimitiveType(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsCommonMatrixTypeThenReturnFalse() {
        // given
        SymbolTestCase testCase = testCaseLibrary.MATRIX_Z_SYMBOL;

        // when
        boolean result = symbolUtil.isPrimitiveType(testCase.getSymbol());

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenSymbolIsAnyUnknownOtherTypeThenReturnFalse() {
        // given
        EMAPortInstanceSymbol symbol = (new SymbolMockBuilder())
                .withPrimitiveDimension()
                .withEmadlType("C")
                .build();

        when(symbol.getTypeReference().getName()).thenReturn("C");

        // when
        boolean result = symbolUtil.isPrimitiveType(symbol);

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenSymbolForPrimitiveTypeIsNullThenThrowNullpointerException() {
        assertThatExceptionOfType(NullPointerException.class)
                .isThrownBy(() -> symbolUtil.isPrimitiveType(null));
    }

    /************************* retrieveEmadlType Tests *************************/

    @Test
    public void whenSymbolIsPrimitiveQTypeThenReturnEmadlTypeQ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Q_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Q);
    }

    @Test
    public void whenSymbolIsPrimitiveZTypeThenReturnEmadlTypeZ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;


        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Z);
    }

    @Test
    public void whenSymbolIsPrimitiveBTypeThenReturnEmadlTypeB() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_B_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.B);
    }

    @Test
    public void whenSymbolIsUnknownTypeThenThrowIllegalArgumentException() {
        // given
        EMAPortInstanceSymbol symbol = (new SymbolMockBuilder())
                .withPrimitiveDimension()
                .withEmadlType("C")
                .build();

        // then
        assertThatExceptionOfType(IllegalArgumentException.class)
                .isThrownBy(() -> symbolUtil.retrieveEmadlType(symbol));
    }

    @Test
    public void whenSymbolIsVectorQTypeThenReturnEmadlTypeQ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.VECTOR_Q_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Q);
    }

    @Test
    public void whenSymbolIsCommonMatrixQTypeThenReturnEmadlTypeQ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.MATRIX_Q_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Q);
    }

    @Test
    public void whenSymbolIsCommonCubeQTypeThenReturnEmadlTypeQ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.CUBE_Q_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Q);
    }

    @Test
    public void whenSymbolIsVectorZTypeThenReturnEmadlTypeZ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.VECTOR_Z_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Z);
    }

    @Test
    public void whenSymbolIsCommonMatrixZTypeThenReturnEmadlTypeZ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.MATRIX_Z_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Z);
    }

    @Test
    public void whenSymbolIsCommonCubeZTypeThenReturnEmadlTypeZ() {
        // given
        SymbolTestCase testCase = testCaseLibrary.CUBE_Z_SYMBOL;

        // when
        EmadlType result = symbolUtil.retrieveEmadlType(testCase.getSymbol());

        // then
        assertThat(result).isEqualTo(EmadlType.Z);
    }

    @Test
    public void whenSymbolIsMultidimensionalUnknownTypeThenThrowIllegalArgumentException() {
        // given
        EMAPortInstanceSymbol symbol = (new SymbolMockBuilder())
                .withCubeDimension()
                .withEmadlType("C")
                .build();

        // when
        assertThatExceptionOfType(IllegalArgumentException.class)
            .isThrownBy(() -> symbolUtil.retrieveEmadlType(symbol));
    }

    /************************* retrieveDimensions Tests *************************/

    @Test
    public void whenSymbolIsVectorDimensionThenReturnListOfSizeOne() {
        // given
        SymbolTestCase testCase = testCaseLibrary.VECTOR_Z_SYMBOL;

        // when
        List<Integer> result = symbolUtil.retrieveDimensions(testCase.getSymbol());

        // then
        assertThat(result).hasSize(1);
        assertThat(result).isEqualTo(testCase.getActualDimension());
    }

    @Test
    public void whenSymbolIsMatrixDimensionThenReturnListOfSizeTwo() {
        // given
        SymbolTestCase testCase = testCaseLibrary.MATRIX_Z_SYMBOL;

        // when
        List<Integer> result = symbolUtil.retrieveDimensions(testCase.getSymbol());

        // then
        assertThat(result).hasSize(2);
        assertThat(result).isEqualTo(testCase.getActualDimension());
    }

    @Test
    public void whenSymbolIsCubeDimensionThenReturnListOfSizeThree() {
        // given
        SymbolTestCase testCase = testCaseLibrary.CUBE_Z_SYMBOL;

        // when
        List<Integer> result = symbolUtil.retrieveDimensions(testCase.getSymbol());

        // then
        assertThat(result).hasSize(3);
        assertThat(result).isEqualTo(testCase.getActualDimension());
    }

    @Test
    public void whenSymbolIsPrimitiveThenReturnListOfSizeOne() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;

        // when
        List<Integer> result = symbolUtil.retrieveDimensions(testCase.getSymbol());

        // then
        assertThat(result).hasSize(1);
        assertThat(result).isEqualTo(testCase.getActualDimension());
    }

    /************************* willAccept Tests *************************/

    @Test
    public void whenSymbolIsPrimitiveQTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Q_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsPrimitiveZTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;


        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsPrimitiveBTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.PRIMITIVE_B_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsUnknownTypeThenWillNotAccept() {
        // given
        EMAPortInstanceSymbol symbol = (new SymbolMockBuilder())
                .withPrimitiveDimension()
                .withEmadlType("C")
                .build();

        // when
        boolean result = symbolUtil.willAccept(symbol);

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenSymbolIsVectorQTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.VECTOR_Q_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsCommonMatrixQTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.MATRIX_Q_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsCommonCubeQTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.CUBE_Q_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsVectorZTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.VECTOR_Z_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsCommonMatrixZTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.MATRIX_Z_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsCommonCubeZTypeThenWillAccept() {
        // given
        SymbolTestCase testCase = testCaseLibrary.CUBE_Z_SYMBOL;

        // when
        boolean result = symbolUtil.willAccept(testCase.getSymbol());

        // then
        assertThat(result).isTrue();
    }

    @Test
    public void whenSymbolIsMultidimensionalUnknownTypeThenWillNotAccept() {
        // given
        EMAPortInstanceSymbol symbol = (new SymbolMockBuilder())
                .withCubeDimension()
                .withEmadlType("C")
                .build();

        // when
        boolean result = symbolUtil.willAccept(symbol);

        // then
        assertThat(result).isFalse();
    }
}
