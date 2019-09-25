/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common;

import org.apache.maven.plugin.Mojo;

public interface PluginContribution {
    /**
     * @return A number indicating at what position the PluginContribution should be executed. Higher means sooner.
     */
    int getPriority();

    /**
     * This method is called once the plugin is being configured.
     * @param plugin The plugin which is being configured.
     * @throws Exception An exception which might occur during configuration.
     */
    default void onPluginConfigure(Mojo plugin) throws Exception {}

    /**
     * This method is called once the plugin is being executed.
     * @param plugin The plugin which is being executed.
     * @throws Exception An exception which might occur during execution.
     */
    default void onPluginExecute(Mojo plugin) throws Exception {}

    /**
     * This method is called once the plugin has finished execution.
     * @param plugin The plugin which has been executed.
     * @throws Exception An exception which might occur during shutdown.
     */
    default void onPluginShutdown(Mojo plugin) throws Exception {}
}
