/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import org.junit.Before;
import org.junit.Test;
import de.monticore.lang.monticar.generator.pythonwrapper.util.SymbolTestCase;
import de.monticore.lang.monticar.generator.pythonwrapper.util.SymbolTestCaseLibrary;
import de.monticore.lang.monticar.generator.pythonwrapper.util.SymbolTestCaseLibraryFactory;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.*;

/**
 *
 */
public class EmadlInstanceSymbol2PortVariableMapperTest {
    private static final String ANY_SYMBOL_NAME = "anyName";
    private static final PortDirection ANY_PORT_DIRECTION = PortDirection.INPUT;
    private static final EmadlType ANY_EMADL_TYPE = EmadlType.Z;
    private static final List<Integer> ANY_MATRIX_DIMENSION = Lists.newArrayList(3,2,4);
    private static final List<Integer> ANY_PRIMITIVE_DIMENSION = Lists.newArrayList(1);

    private static final PortVariable PRIMITIVE_PORT_VARIABLE
            = PortVariable.primitiveVariableFrom(ANY_SYMBOL_NAME, ANY_EMADL_TYPE, ANY_PORT_DIRECTION);

    private static final PortVariable MATRIX_PORT_VARIABLE
            = PortVariable.multidimensionalVariableFrom(ANY_SYMBOL_NAME, ANY_EMADL_TYPE, ANY_PORT_DIRECTION, ANY_MATRIX_DIMENSION);

    private EmadlInstanceSymbol2PortVariableMapper portMapper;

    private EmadlInstanceSymbolUtil emadlInstanceSymbolUtil;
    private SymbolTestCaseLibrary testCaseLibrary;

    @Before
    public void setup() {
        emadlInstanceSymbolUtil = mock(EmadlInstanceSymbolUtil.class, RETURNS_DEEP_STUBS);

        when(emadlInstanceSymbolUtil.retrieveSymbolName(any())).thenReturn(ANY_SYMBOL_NAME);
        when(emadlInstanceSymbolUtil.retrievePortType(any())).thenReturn(ANY_PORT_DIRECTION);
        when(emadlInstanceSymbolUtil.retrieveEmadlType(any())).thenReturn(ANY_EMADL_TYPE);

        when(emadlInstanceSymbolUtil.willAccept(any())).thenReturn(true);

        portMapper = new EmadlInstanceSymbol2PortVariableMapper(emadlInstanceSymbolUtil);

        SymbolTestCaseLibraryFactory factory = new SymbolTestCaseLibraryFactory();
        testCaseLibrary = factory.getSymbolTestCaseLibrary();
    }

    @Test
    public void whenPrimitiveTypeIsGivenThenPrimitivePortVariableIsReturned() {
        // given
        SymbolTestCase symbolTestCase = testCaseLibrary.PRIMITIVE_Z_SYMBOL;
        EMAPortInstanceSymbol symbol = symbolTestCase.getSymbol();

        when(emadlInstanceSymbolUtil.isPrimitiveType(symbol)).thenReturn(true);
        when(emadlInstanceSymbolUtil.isCommonMatrixType(symbol)).thenReturn(false);
        when(emadlInstanceSymbolUtil.retrieveDimensions(symbol)).thenReturn(ANY_PRIMITIVE_DIMENSION);

        // when
        PortVariable result = portMapper.map(symbolTestCase.getSymbol());

        // then
        assertThat(result).isEqualTo(PRIMITIVE_PORT_VARIABLE);
    }

    @Test
    public void whenMatrixTypeIsGivenThenMultidimensionalTypeOfBuilderIsReturned() {
        // given
        SymbolTestCase symbolTestCase = testCaseLibrary.MATRIX_Z_SYMBOL;
        EMAPortInstanceSymbol symbol = symbolTestCase.getSymbol();

        when(emadlInstanceSymbolUtil.retrieveSymbolName(symbol)).thenReturn(ANY_SYMBOL_NAME);
        when(emadlInstanceSymbolUtil.isPrimitiveType(symbol)).thenReturn(false);
        when(emadlInstanceSymbolUtil.isCommonMatrixType(symbol)).thenReturn(true);
        when(emadlInstanceSymbolUtil.retrieveDimensions(symbol)).thenReturn(ANY_MATRIX_DIMENSION);

        // when
        PortVariable result = portMapper.map(symbolTestCase.getSymbol());

        // then
        assertThat(result).isEqualTo(MATRIX_PORT_VARIABLE);
    }

    @Test
    public void whenSymbolIsNullThenWillNotAccept() {
        // when
        boolean result = portMapper.willAccept(null);

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenSymbolIsNotNullAndUtilWillNotAcceptThenWillNotAccept() {
        // given
        EMAPortInstanceSymbol anySymbol = testCaseLibrary.MATRIX_Z_SYMBOL.getSymbol();
        when(emadlInstanceSymbolUtil.willAccept(anySymbol)).thenReturn(false);

        // when
        boolean result = portMapper.willAccept(anySymbol);

        // then
        assertThat(result).isFalse();
    }

    @Test
    public void whenSymbolIsNotNullAndUtilWillAcceptThenWillAccept() {
        // given
        EMAPortInstanceSymbol anySymbol = testCaseLibrary.MATRIX_Z_SYMBOL.getSymbol();

        // when
        boolean result = portMapper.willAccept(anySymbol);

        // then
        assertThat(result).isTrue();
    }
}
