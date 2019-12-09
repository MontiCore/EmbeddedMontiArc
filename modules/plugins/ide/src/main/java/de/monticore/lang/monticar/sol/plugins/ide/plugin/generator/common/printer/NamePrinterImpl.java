/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.symboltable.Symbol;

import javax.annotation.Nullable;
import java.util.Optional;

@Singleton
public class NamePrinterImpl extends IDEPrinterImpl implements NamePrinter {
    protected static final String ERROR = "<There seems to be an issue>";

    @Inject
    protected NamePrinterImpl(GeneratorEngine engine) {
        super(engine);
    }

    @Override
    public String printMethodNameSuffix(@Nullable Symbol symbol) {
        String name = Optional.ofNullable(symbol).map(Symbol::getName).orElse(ERROR);

        return name.substring(0, 1).toUpperCase() + name.substring(1);
    }

    @Override
    public String printQualifiedClassPrefix(@Nullable Symbol symbol) {
        String name = Optional.ofNullable(symbol).map(Symbol::getFullName).orElse(ERROR);

        return name.replaceAll("\\.", "\\$");
    }

    @Override
    public String printQualifiedPath(@Nullable Symbol symbol) {
        String name = Optional.ofNullable(symbol).map(Symbol::getFullName).orElse(ERROR);

        return name.toLowerCase();
    }
}
