package de.monticore.lang.monticar.borealis.runtime.language.ls;

public interface ServerLauncher {
    void initialize(String[] arguments) throws Exception;
    void configure(String[] arguments) throws Exception;
    void launch(String[] arguments) throws Exception;
}
