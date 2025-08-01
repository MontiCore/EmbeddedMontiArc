/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.underspecification;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSpecificationSymbol;

public class HandleUnderSpecification {


    public static boolean isUnderspecified(EMAMSpecificationSymbol specification) {
        return specification.getVariables().size() > specification.getEquations().size();
    }
}
