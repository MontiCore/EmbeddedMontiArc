/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._cocos;

/**
 * Collection of context conditions (cocos) for MontiMathOpt
 */
public class MathOptCocos {

    public static MathOptCoCoChecker createChecker() {
        return new MathOptCoCoChecker()
                .addCoCo(new OptimizationStatementCheck())
                .addCoCo(new OptimizationConditionCheck());
    }
}
