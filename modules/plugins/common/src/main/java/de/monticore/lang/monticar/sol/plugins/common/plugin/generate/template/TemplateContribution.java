/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template;

import java.nio.file.Path;

public interface TemplateContribution { // TODO: Rewrite this garbage.
    /**
     * @return True if this template has a hand-coded peer, false otherwise.
     */
    boolean hasHandCodedPeer();

    /**
     * @return The relative path to the template file (e.g. templates/language-server/LanguageModule.ftl).
     */
    String getTemplatePath();

    /**
     * @return The relative path to the output file (e.g ls/CarLangLanguageServer.java).
     */
    Path getOutputPath();

    /**
     * @return The relative path to the top output file (e.g ls/CarLangLanguageServerTop.java).
     */
    default Path getTopPatternOutputFile() throws Exception {
        throw new Exception("This template does not support the Top Pattern!");
    }
}
