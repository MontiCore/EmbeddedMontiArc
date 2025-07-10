/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common;

import com.google.inject.Module;

public interface Plugin {
    /**
     * @return The module in which the bindings are configured.
     */
    Module getModule();
}
