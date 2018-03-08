package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class IndexHelper {
    private IndexHelper() {
    }

    public static List<String> getDimSizesOfMatrixType(ASTCommonMatrixType matrixType) {
        List<String> res = new ArrayList<>();
        for (ASTCommonDimensionElement cde : matrixType.getCommonDimension().getCommonDimensionElements()) {
            if (cde.unitNumberIsPresent()) {
                res.add("" + cde.getUnitNumber().get().getNumber().get().longValue());
            } else if (cde.nameIsPresent()) {
                res.add(cde.getName().get());
            } else {
                Log.error("Matrix dimensions are not clearly defined.");
            }

        }
        return res;
    }
}
