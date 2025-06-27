/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.EmadlType;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.types.types._ast.ASTType;

import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;

/**
 *
 */
class EmadlInstanceSymbolUtil {
    private final static String COMMON_MATRIX_TYPE_STRING = "CommonMatrixType";

    private final static Set<String> PRIMITIVE_EMADL_STRING_TYPES = Sets.newHashSet("Q", "Z", "B");

    String retrieveSymbolName(final EMAPortInstanceSymbol symbol) {
        checkNotNull(symbol);

        return symbol.getName();
    }

    public PortDirection retrievePortType(final EMAPortInstanceSymbol symbol) {
        checkNotNull(symbol);

        return symbol.isIncoming() ? PortDirection.INPUT : PortDirection.OUTPUT;
    }

    public boolean isCommonMatrixType(final EMAPortInstanceSymbol symbol) {
        checkNotNull(symbol);

        return symbol.getTypeReference().getName().equals(COMMON_MATRIX_TYPE_STRING);
    }

    public boolean isPrimitiveType(final EMAPortInstanceSymbol symbol) {
        checkNotNull(symbol);

        return symbol.getTypeReference().getName().equals("Q")
                || symbol.getTypeReference().getName().equals("Z")
                || symbol.getTypeReference().getName().equals("B");
    }

    private void checkIfEitherPrimitiveOrCommonMatrixType(final EMAPortInstanceSymbol symbol) {
        checkNotNull(symbol);
        checkArgument(isCommonMatrixType(symbol) || isPrimitiveType(symbol));
    }

    public EmadlType retrieveEmadlType(final EMAPortInstanceSymbol symbol) {
        checkIfEitherPrimitiveOrCommonMatrixType(symbol);
        String nameOfType;
        if (isCommonMatrixType(symbol)) {
            nameOfType = retrieveMatrixType(symbol).getElementType().getName();
        } else {
            nameOfType = retrieveTypeReference(symbol).getName();
        }
        return EmadlType.fromString(nameOfType);
    }

    public List<Integer> retrieveDimensions(final EMAPortInstanceSymbol symbol) {
        checkIfEitherPrimitiveOrCommonMatrixType(symbol);
        List<Integer> dimensions;
        if (isCommonMatrixType(symbol)) {
            ASTCommonMatrixType matrixType = retrieveMatrixType(symbol);
            dimensions = matrixType.getDimension().getMatrixDimList().stream()
                    .map(this::castToAstNumberExpression)
                    .map(this::retrieveIntegerFromAstNumberExpression)
                    .collect(Collectors.toList());
        } else {
            dimensions = Collections.singletonList(1);
        }
        return dimensions;
    }

    private ASTNumberExpression castToAstNumberExpression(final ASTExpression astExpression) {
        if (!(astExpression instanceof ASTNumberExpression)) {
            throw new IllegalStateException("Unknown Expression for dimension");
        }
        return ((ASTNumberExpression) astExpression);
    }

    private Integer retrieveIntegerFromAstNumberExpression(final ASTNumberExpression astNumberExpression) {
        return astNumberExpression.getNumberWithUnit().getNumber().orElseThrow(IllegalArgumentException::new).intValue();
    }

    private MCTypeReference retrieveTypeReference(final EMAPortInstanceSymbol symbol) {
        return symbol.getTypeReference();
    }

    private ASTCommonMatrixType retrieveMatrixType(final EMAPortInstanceSymbol symbol) {
        checkArgument(isCommonMatrixType(symbol), "Symbol is not a CommonMatrixType");

        MCTypeReference typeReference = retrieveTypeReference(symbol);
        if (!(typeReference instanceof MCASTTypeSymbolReference)) {
            throw new IllegalArgumentException("Symbol has no MCASTTypeSymbolReference type reference");
        }
        ASTType astType = ((MCASTTypeSymbolReference) typeReference).getAstType();

        if(!(astType instanceof ASTCommonMatrixType)) {
            throw new IllegalArgumentException("Symbol has no ASTCommonMatrixType ASTType");
        }
        return ((ASTCommonMatrixType) astType);
    }

    public boolean willAccept(final EMAPortInstanceSymbol symbol) {
        boolean accept =  checkSymbolName(symbol) && checkTypeReference(symbol);

        if (accept && isCommonMatrixType(symbol)) {
            if (!checkAstCommonMatrixTypeCanBeRetrieved(symbol)) {
                return false;
            }

            ASTCommonMatrixType astCommonMatrixType = retrieveMatrixType(symbol);
            return checkEmadlTypeOfAstCommonMatrixTypeCanBeRetrieved(astCommonMatrixType)
                    && checkDimensionOfAstCommonMatrixTypeCanBeRetrieved(astCommonMatrixType)
                    && retrieveDimensions(symbol).size() > 0
                    && retrieveDimensions(symbol).size() <= 3;
        }

        return accept;
    }

    private boolean checkEmadlTypeOfAstCommonMatrixTypeCanBeRetrieved(final ASTCommonMatrixType astCommonMatrixType) {
        return astCommonMatrixType.getElementType() != null
                && astCommonMatrixType.getElementType().getName() != null
                && PRIMITIVE_EMADL_STRING_TYPES.contains(astCommonMatrixType.getElementType().getName());
    }

    private boolean checkDimensionOfAstCommonMatrixTypeCanBeRetrieved(final ASTCommonMatrixType astCommonMatrixType) {
        return astCommonMatrixType.getDimension() != null
                && astCommonMatrixType.getDimension().getMatrixDimList() != null
                && astCommonMatrixType.getDimension().getMatrixDimList().stream().allMatch(e -> e instanceof ASTNumberExpression)
                && astCommonMatrixType.getDimension().getMatrixDimList().stream()
                .map(this::castToAstNumberExpression)
                .allMatch(e -> e.getNumberWithUnit() != null
                        && e.getNumberWithUnit().getNumber() != null
                        && e.getNumberWithUnit().getNumber().isPresent());
    }

    private boolean checkAstCommonMatrixTypeCanBeRetrieved(final EMAPortInstanceSymbol symbol) {
        return isCommonMatrixType(symbol)
                && retrieveTypeReference(symbol) instanceof MCASTTypeSymbolReference
                && (((MCASTTypeSymbolReference)retrieveTypeReference(symbol)).getAstType() instanceof ASTCommonMatrixType);
    }

    private boolean checkTypeReference(final EMAPortInstanceSymbol symbol) {
        return symbol.getTypeReference() != null
                && Lists.newArrayList("Q", "B", "Z", COMMON_MATRIX_TYPE_STRING)
                    .contains(symbol.getTypeReference().getName());
    }

    private boolean checkSymbolName(final EMAPortInstanceSymbol symbol) {
        return symbol != null
                && retrieveSymbolName(symbol) != null
                && !retrieveSymbolName(symbol).isEmpty();
    }
}
