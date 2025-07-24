/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.printer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValue;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValueList;

import java.util.List;
import java.util.stream.Collectors;

@Singleton
public class CommonPrinterImpl implements CommonPrinter {
    protected static final String ERROR = "// Something went wrong";

    protected final String radix;
    protected final GeneratorEngine engine;

    @Inject
    protected CommonPrinterImpl(GeneratorEngine engine) {
        this(engine, "templates/common");
    }

    protected CommonPrinterImpl(GeneratorEngine engine, String radix) {
        this.engine = engine;
        this.radix = radix;
    }

    protected String getTemplatePath(String suffix) {
        return String.format("%s/%s.ftl", this.radix, suffix);
    }

    protected String generate(String suffix, Object... arguments) {
        return this.engine.generateNoA(this.getTemplatePath(suffix), arguments).toString();
    }

    @Override
    public String print(SymbolWithValue symbol) {
        String value = symbol.getValue().map(Object::toString).orElse(ERROR);

        if (symbol.isString()) return String.format("`%s`", value);
        else return value;
    }

    @Override
    public String print(SymbolWithValueList symbol) {
        List<String> values = symbol.getValues().stream()
                .map(Object::toString)
                .collect(Collectors.toList());

        values = symbol.isStringList() ? this.quoteStrings(values) : values;

        return String.format("[%s]", String.join(", ", values));
    }

    protected List<String> quoteStrings(List<String> strings) {
        return strings.stream()
                .map(string -> String.format("`%s`", string))
                .collect(Collectors.toList());
    }
}
