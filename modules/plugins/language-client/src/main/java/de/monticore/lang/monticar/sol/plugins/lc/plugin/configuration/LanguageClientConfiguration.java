/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;

import java.util.List;

public interface LanguageClientConfiguration extends GeneratePluginConfiguration {
    /**
     * @return The qualified name of the grammar.
     */
    String getGrammarQualifiedName();

    /**
     * @return The name of the grammar.
     */
    String getGrammarName();

    /**
     * @return The file extension that the model files will have.
     */
    String getFileExtension();

    /**
     * @return The keywords which will be excluded from Syntax Highlighting.
     */
    List<String> getExcludedKeywords();
}
