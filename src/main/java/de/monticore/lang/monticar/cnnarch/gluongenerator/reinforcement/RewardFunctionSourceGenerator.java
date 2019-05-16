package de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement;

/**
 *
 */
public interface RewardFunctionSourceGenerator {
    void generate(String modelPath, String qualifiedName, String targetPath);
}