/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.application;

public interface ApplicationContribution {
    default void configure(Application application) throws Exception {}
    default void prepare(Application application) throws Exception {}
    default void start(Application application, String[] args) throws Exception {}
    default void stop(Application application) throws Exception {}
}
