/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._cocos;

/**
 * creates the abstract math coco checker and add all specific checkers
 */
public class MathCocos {

    public static MathCoCoChecker createChecker() {
        return new MathCoCoChecker()
                .addCoCo(new MatrixAssignmentDeclarationCheck())
                .addCoCo(new MatrixAssignmentCheck());
                    //.addCoCo(new DimensionEquals())
                    //.addCoCo(new ArithmeticMatrixExpressionCheck())
                    //.addCoCo(new MatrixUnitCheck())
                    //.addCoCo(new ArithmeticExpressionCheck())
                    //.addCoCo(new RangeCheck());
    }
}
