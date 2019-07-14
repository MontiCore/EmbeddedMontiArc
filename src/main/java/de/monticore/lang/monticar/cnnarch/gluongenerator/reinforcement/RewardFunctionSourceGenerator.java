package de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

/**
 *
 */
public interface RewardFunctionSourceGenerator {
    EMAComponentInstanceSymbol generate(String modelPath, String qualifiedName, String targetPath);
}