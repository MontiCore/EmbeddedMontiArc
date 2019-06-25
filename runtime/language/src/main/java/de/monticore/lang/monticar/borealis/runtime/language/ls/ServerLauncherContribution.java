package de.monticore.lang.monticar.borealis.runtime.language.ls;

public interface ServerLauncherContribution {
    default void onInitialize(String[] arguments) throws Exception {}
    default void onConfigure(String[] arguments) throws Exception {}
}
