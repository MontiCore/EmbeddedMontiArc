/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._ast;

import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;

import java.util.List;
import java.util.Optional;

public class ASTArchSpecialArgument extends ASTArchSpecialArgumentTOP {

    public ASTArchSpecialArgument() {
    }

    public ASTArchSpecialArgument(ASTArchExpression rhs, Optional<String> serial, Optional<String> parallel, Optional<String> conditional) {
        super(rhs, serial, parallel, conditional);
    }

    @Override
    public String getName() {
        if (isPresentParallel()){
            return AllPredefinedVariables.PARALLEL_ARG_NAME;
        }
        else if (isPresentSerial()) {
            return AllPredefinedVariables.SERIAL_ARG_NAME;
        }
        else if (isPresentConditional()){
            return AllPredefinedVariables.CONDITIONAL_ARG_NAME;
        }
        else {
            throw new IllegalStateException();
        }
    }

}
