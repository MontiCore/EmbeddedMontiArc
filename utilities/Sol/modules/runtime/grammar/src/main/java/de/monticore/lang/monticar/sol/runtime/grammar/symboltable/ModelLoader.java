/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.grammar.symboltable;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.symboltable.ResolvingConfiguration;

/**
 * An interface to be implemented by ModelLoaders.
 */
public interface ModelLoader {
    /**
     * Initializes a new instance of ModelingLanguageFamily to be used in the ModelLoader.
     * @param languages The languages to be included.
     * @return An instantiated instance of the ModelingLanguageFamily.
     */
    default ModelingLanguageFamily initModelingLanguageFamily(ModelingLanguage...languages) {
        ModelingLanguageFamily family = new ModelingLanguageFamily();

        for (ModelingLanguage language : languages) {
            family.addModelingLanguage(language);
        }

        return family;
    }

    /**
     * Initialized a new instance of ResolvingConfiguration to be used in the ModelLoader.
     * @param languages The languages to be included.
     * @return An instantiated instance of the ResolvingConfiguration.
     */
    default ResolvingConfiguration initResolvingConfiguration(ModelingLanguage ...languages) {
        ResolvingConfiguration configuration = new ResolvingConfiguration();

        for (ModelingLanguage language : languages) {
            configuration.addDefaultFilters(language.getResolvingFilters());
        }

        return configuration;
    }
}
