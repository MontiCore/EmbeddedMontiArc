/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.AbstractPlugin;

import java.io.File;
import java.util.List;

public abstract class AbstractGeneratePlugin extends AbstractPlugin {
    /**
     * @return The output directory.
     */
    public abstract File getOutputPath();

    /**
     * @return The directories which hold handwritten code.
     */
    public abstract List<File> getHandCodedPaths();
}
