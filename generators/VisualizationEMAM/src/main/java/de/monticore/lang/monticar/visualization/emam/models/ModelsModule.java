/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.models;

import com.google.inject.AbstractModule;

public class ModelsModule extends AbstractModule {
    @Override
    public void configure() {
        bind(ModelPathVisitor.class).to(ModelPathVisitorImpl.class);
        bind(ModelSplitter.class).to(ModelSplitterImpl.class);
    }
}
