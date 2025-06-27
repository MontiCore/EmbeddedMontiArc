/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.Constraints;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterType;

public class AllPredefinedVariables {

    public static final String CONDITIONAL_ARG_NAME = "?";
    public static final String SERIAL_ARG_NAME = "->";
    public static final String PARALLEL_ARG_NAME = "|";
    public static final String TRUE_NAME = "true";
    public static final String FALSE_NAME = "false";

    public static ParameterSymbol createConditionalParameter(){
        return new ParameterSymbol.Builder()
                .name(CONDITIONAL_ARG_NAME)
                .type(ParameterType.LAYER_PARAMETER)
                .constraints(Constraints.BOOLEAN)
                .defaultValue(true)
                .build();
    }

    public static ParameterSymbol createSerialParameter(){
        return new ParameterSymbol.Builder()
                .name(SERIAL_ARG_NAME)
                .type(ParameterType.LAYER_PARAMETER)
                .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                .defaultValue(1)
                .build();
    }

    public static ParameterSymbol createParallelParameter(){
        return new ParameterSymbol.Builder()
                .name(PARALLEL_ARG_NAME)
                .type(ParameterType.LAYER_PARAMETER)
                .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                .defaultValue(1)
                .build();
    }

    //necessary because true is currently only a name in MontiMath and it needs to be evaluated at compile time for this language
    public static ParameterSymbol createTrueConstant(){
        return new ParameterSymbol.Builder()
                .name(TRUE_NAME)
                .type(ParameterType.CONSTANT)
                .defaultValue(true)
                .build();
    }

    //necessary because false is currently only a name in MontiMath and it needs to be evaluated at compile time for this language
    public static ParameterSymbol createFalseConstant() {
        return new ParameterSymbol.Builder()
                .name(FALSE_NAME)
                .type(ParameterType.CONSTANT)
                .defaultValue(false)
                .build();
    }
}
