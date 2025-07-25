/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

import java.nio.file.Path;

public interface Template {
    /**
     * @return Relative path to the freemarker template.
     */
    String getTemplatePath();

    /**
     * @return Resolved relative path to the file which should be generated.
     */
    Path getOutputPath();

    /**
     * @return Resolved relative path to the file which should be generated when handwritten code exists.
     */
    Path getTopPatternOutputPath();

    /**
     * @return True if the template has a handwritten peer, false otherwise.
     */
    boolean hasHandwrittenPeer();

    /**
     * @return Arguments to be passed to the GeneratorEngine.
     */
    Object[] getArguments();
}
