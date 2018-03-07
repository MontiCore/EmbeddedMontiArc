package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import org.jscience.mathematics.number.Rational;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class IndexHelper {
    private IndexHelper() {
    }

    public static List<String> getIndexStrings(List<Long> strides) {
        List<String> indexStrings = new ArrayList<>();
        Long size = strides.get(0);
        for (long i = 0; i < size; i++) {
            long curI = i;
            List<Long> curIndices = new ArrayList<>();
            for (int j = 1; j < strides.size(); j++) {
                Long curStride = strides.get(j);
                curIndices.add(curI / curStride);
                curI = curI % curStride;
            }
            curIndices.add(curI);

            indexStrings.add(curIndices.stream()
                    .map(Object::toString)
                    .collect(Collectors.joining(", ")));

        }
        return indexStrings;
    }

    public static List<Long> getDimSizesOfMatrixType(ASTCommonMatrixType matrixType) {
        return matrixType.getCommonDimension().getCommonDimensionElements().stream()
                .map(ASTCommonDimensionElement::getUnitNumber)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .map(ASTUnitNumber::getNumber)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .map(Rational::longValue)
                .collect(Collectors.toList());
    }

    public static List<Long> getStridesOfDimSizes(List<Long> dimSizes) {
        List<Long> strides = new ArrayList<>();

        for (int i = 0; i < dimSizes.size(); i++) {
            strides.add(1L);
        }

        for (int i = 0; i < dimSizes.size(); i++) {
            for (int j = dimSizes.size() - 1; j >= i; j--) {
                strides.set(i, strides.get(i) * dimSizes.get(j));
            }
        }
        return strides;
    }

    public static List<String> getIndexStrings(ASTCommonMatrixType matrixType) {
        return getIndexStrings(getStridesOfDimSizes(getDimSizesOfMatrixType(matrixType)));
    }
}
