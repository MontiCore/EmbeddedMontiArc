/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator.serializer;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;

import java.util.function.Consumer;

@Singleton
public class OptionsSerializerImpl implements OptionsSerializer {
    protected final Gson gson;

    @Inject
    protected OptionsSerializerImpl(Gson gson) {
        this.gson = gson;
    }

    @Override
    public String serialize(SymbolWithOptions symbol) {
        JsonArray result = new JsonArray();
        Consumer<OptionSymbol> consumer = option -> result.add(this.gson.toJsonTree(option));

        symbol.getOptionSymbols().forEach(consumer);
        return this.gson.toJson(result);
    }
}
