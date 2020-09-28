/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;


import de.monticore.symboltable.Symbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Contains all information that is relevant for the target language to generate the source code.
 *
 */
public abstract class LanguageUnit {

    protected List<EMAMBluePrint> bluePrints = new ArrayList<>();
    protected List<Symbol> symbolsToConvert = new ArrayList<>();

    public List<EMAMBluePrint> getBluePrints() {
        return bluePrints;
    }

    public void addSymbolToConvert(Symbol symbol) {
        symbolsToConvert.add(symbol);
    }

    public Optional<EMAMBluePrint> getBluePrint(String fullName) {
        for (EMAMBluePrint bluePrint : bluePrints) {
            if (bluePrint.getName().equals(fullName.replaceAll("\\.", "_"))) {
                return Optional.of(bluePrint);
            }
        }
        return Optional.empty();
    }

    public abstract void generateBluePrints();

}
