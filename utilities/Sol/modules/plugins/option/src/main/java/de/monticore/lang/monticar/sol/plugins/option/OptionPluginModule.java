/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option.serializer.OptionSerializer;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionGlex;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionMethodDelegator;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionMethodDelegatorImpl;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.OptionPrinter;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.OptionPrinterImpl;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinterImpl;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.serializer.OptionsSerializer;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.serializer.OptionsSerializerImpl;

public class OptionPluginModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
    }

    private void addBindings() {
        bind(OptionPrinter.class).to(OptionPrinterImpl.class);
        bind(ValidatorPrinter.class).to(ValidatorPrinterImpl.class);
        bind(OptionMethodDelegator.class).to(OptionMethodDelegatorImpl.class);
        bind(OptionsSerializer.class).to(OptionsSerializerImpl.class);
    }

    private void addMultiBindings() {
        this.addGlexContributions();
    }

    private void addGlexContributions() {
        Multibinder<GlexContribution> contributions =
                Multibinder.newSetBinder(binder(), GlexContribution.class);

        contributions.addBinding().to(OptionGlex.class);
    }

    @Provides
    protected Gson provideGson(OptionSerializer serializer) {
        GsonBuilder builder = new GsonBuilder();

        return builder.registerTypeAdapter(OptionSymbol.class, serializer).setPrettyPrinting().create();
    }
}
