/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstanceInformation;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Ahmed Diab
 */
public class MathExpressionProperties {
    public Level pre = Level.ARMA;
    public Level succ = Level.ARMA;

    enum Level {
        CV,
        ARMA
    }

    /*public MathExpressionProperties(String name) {
        (name);
    }*/
}
