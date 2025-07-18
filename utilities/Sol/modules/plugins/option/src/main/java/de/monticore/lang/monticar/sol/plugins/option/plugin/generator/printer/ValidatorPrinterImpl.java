/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;
import de.monticore.prettyprint.IndentPrinter;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.stream.Collectors;

@Singleton
public class ValidatorPrinterImpl extends OptionPrinterImpl implements ValidatorPrinter {
    protected final Deque<OptionSymbol> hierarchy;
    protected final IndentPrinter printer;

    protected String suffix;
    protected SymbolWithOptions symbol;

    @Inject
    protected ValidatorPrinterImpl(GeneratorEngine engine) {
        super(engine);

        this.hierarchy = new ArrayDeque<>();
        this.printer = new IndentPrinter();

        this.printer.setIndentLength(4);
    }

    @Override
    public String printMethods(SymbolWithOptions symbol) {
        this.hierarchy.clear();
        this.printer.clearBuffer();
        this.printer.indent();
        return this.generate("validators/methods", symbol);
    }

    @Override
    public String printMethods(SymbolWithOptions symbol, OptionSymbol option) {
        this.symbol = symbol;

        this.hierarchy.clear();
        this.printer.clearBuffer();
        this.printSubMethods(symbol, option);

        return this.printer.getContent();
    }

    protected void printSubMethods(SymbolWithOptions symbol, OptionSymbol option) {
        this.hierarchy.push(option);
        this.recalculateSuffix();
        this.printer.print(this.generate("validators/method/method", symbol, option, this.suffix));
        option.getOptionSymbols().forEach(subOption -> this.printSubMethods(symbol, subOption));
        this.hierarchy.pop();
        this.recalculateSuffix();
    }

    @Override
    public String printBody(OptionSymbol option) {
        return option.getTypeSymbol().map(type -> this.doPrintBody(option, type)).orElse(ERROR);
    }

    protected String doPrintBody(OptionSymbol option, OptionTypeSymbol type) {
        if (type.isArray()) return this.generate("validators/method/body/array", option, this.suffix);
        else if (type.isObject()) return this.generate("validators/method/body/object", option, this.suffix);
        else return this.generate("validators/method/body/returns");
    }

    protected void recalculateSuffix() {
        List<String> capitalizedNames = this.hierarchy.stream()
                .map(OptionSymbol::getName)
                .map(name -> name.substring(0, 1).toUpperCase() + name.substring(1))
                .collect(Collectors.toList());

        Collections.reverse(capitalizedNames);

        this.suffix = String.join("", capitalizedNames);
    }
}
