/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;
import de.monticore.lang.monticar.cnntrain._ast.ASTAdamWOptimizer;

public class CNNArch2GluonTrainParamSupportChecker extends TrainParamSupportChecker {
    public void visit(ASTAdamWOptimizer node){}
}
