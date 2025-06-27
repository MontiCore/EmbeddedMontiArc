/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.reinforcement;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

/**
 *
 */
public interface RewardFunctionSourceGenerator {
    TaggingResolver createTaggingResolver(String modelPath);
    EMAComponentInstanceSymbol resolveSymbol(TaggingResolver taggingResolver, String rootModel);
    void generate(String modelPath, String rootModel, String targetPath);
    void generate(EMAComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver, String targetPath);
}