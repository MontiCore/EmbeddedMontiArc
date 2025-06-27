/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import com.google.common.collect.Lists;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static org.mockito.Mockito.*;

/**
 *
 */
public class SymbolMockBuilder {
    private enum DimensionalType {
        VECTOR, MATRIX, CUBE, PRIMITIVE
    }

    private String name;
    private String emadlType;
    private DimensionalType dimensionalType;
    private List<Integer> dimension;
    private Boolean isIncoming;

    public SymbolMockBuilder withSymbolName(final String name) {
        this.name = name;
        return this;
    }

    public SymbolMockBuilder withIncomingPort() {
        this.isIncoming = true;
        return this;
    }

    public SymbolMockBuilder withOutgoingPort() {
        this.isIncoming = false;
        return this;
    }

    public SymbolMockBuilder withEmadlType(final EmadlType emadlType) {
        this.emadlType = emadlType.toString();
        return this;
    }

    public SymbolMockBuilder withEmadlType(final String type) {
        this.emadlType = type;
        return this;
    }

    public SymbolMockBuilder withVectorDimension() {
        this.dimensionalType = DimensionalType.VECTOR;
        this.dimension = Lists.newArrayList(6);
        return this;
    }

    public SymbolMockBuilder withMatrixDimension() {
        this.dimensionalType = DimensionalType.MATRIX;
        this.dimension = Lists.newArrayList(3,4);
        return this;
    }

    public SymbolMockBuilder withCubeDimension() {
        this.dimensionalType = DimensionalType.CUBE;
        this.dimension = Lists.newArrayList(5,4,6);
        return this;
    }

    public SymbolMockBuilder withPrimitiveDimension() {
        this.dimensionalType = DimensionalType.PRIMITIVE;
        this.dimension = Lists.newArrayList(1);
        return this;
    }

    public SymbolMockBuilder withAnyCommonMatrixType() {
        this.withMatrixDimension();
        return this;
    }

    public SymbolMockBuilder withAnyPrimitiveType() {
        this.withPrimitiveDimension();
        return this;
    }

    public SymbolMockBuilder withDimension(Integer... dims) {
        final int degree = dims.length;
        switch(degree) {
            case 0:
                throw new IllegalArgumentException("Type must be at least one dimensional");
            case 1:
                this.withVectorDimension();
                break;
            case 2:
                this.withMatrixDimension();
                break;
            case 3:
                this.withCubeDimension();
                break;
            default:
                throw new IllegalArgumentException("Type dimensions must be less or equal three");
        }
        this.dimension = Lists.newArrayList(dims);
        return this;
    }

    public EMAPortInstanceSymbol build() {
        if (this.name == null) {
            this.withSymbolName("anyTypeName");
        }

        if (this.isIncoming == null) {
            this.withIncomingPort();
        }

        if (this.dimensionalType == null) {
            this.withPrimitiveDimension();
            this.dimension = null;
        }

        if (this.emadlType == null) {
            this.withEmadlType(EmadlType.Z);
        }

        EMAPortInstanceSymbol symbol = mock(EMAPortInstanceSymbol.class, RETURNS_DEEP_STUBS);

        when(symbol.getName()).thenReturn(this.name);
        when(symbol.isIncoming()).thenReturn(this.isIncoming);

        if (this.dimensionalType.equals(DimensionalType.PRIMITIVE)) {
            MCTypeReference typeReference = mock(MCTypeReference.class, RETURNS_DEEP_STUBS);
            when(symbol.getTypeReference()).thenReturn(typeReference);
            when(typeReference.getName()).thenReturn(this.emadlType);
        } else {
            MCTypeReference typeReference = mock(MCASTTypeSymbolReference.class, RETURNS_DEEP_STUBS);
            when(symbol.getTypeReference()).thenReturn(typeReference);
            ASTCommonMatrixType astCommonMatrixType = mock(ASTCommonMatrixType.class, RETURNS_DEEP_STUBS);
            when(typeReference.getName()).thenReturn("CommonMatrixType");
            when(((MCASTTypeSymbolReference)typeReference).getAstType()).thenReturn(astCommonMatrixType);
            when(astCommonMatrixType.getElementType().getName()).thenReturn(this.emadlType);

            List<ASTExpression> dimensionExpressionList = new ArrayList<>();
            for (int i : this.dimension) {
                ASTNumberExpression numberExpression = mock(ASTNumberExpression.class, RETURNS_DEEP_STUBS);
                Optional<Double> number = Optional.of((double) i);
                when(numberExpression.getNumberWithUnit().getNumber()).thenReturn(number);
                dimensionExpressionList.add(numberExpression);
            }
            when(astCommonMatrixType.getDimension().getMatrixDimList()).thenReturn(dimensionExpressionList);
        }

        return symbol;
    }
}
