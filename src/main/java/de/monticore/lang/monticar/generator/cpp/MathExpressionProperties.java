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
    public Level suc = Level.ARMA;

    enum Level {
        CV,
        ARMA
    }

    public void setPreToCV(){
        this.pre = Level.CV;
    }

    public void setSucToCV(){
        this.suc = Level.CV;
    }

    public boolean isPreCV(){
        return this.pre == Level.CV;
    }

    public boolean isSucCV(){
        return this.suc == Level.CV;
    }
}
