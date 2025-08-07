/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.generator;

import com.google.inject.AbstractModule;

public class GeneratorModule extends AbstractModule {
    @Override
    public void configure() {
        bind(ScriptGenerator.class).to(ScriptGeneratorImpl.class);
        bind(ScriptGeneratorHelper.class).to(ScriptGeneratorHelperImpl.class);

        bind(HTMLGenerator.class).to(HTMLGeneratorImpl.class);
        bind(HTMLGeneratorHelper.class).to(HTMLGeneratorHelperImpl.class);
    }
}
