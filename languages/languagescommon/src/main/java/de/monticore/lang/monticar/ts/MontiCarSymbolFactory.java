/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;

public class MontiCarSymbolFactory implements MCTypeFactory<MontiCarTypeSymbol> {

    public MontiCarFieldSymbol createFormalParameterSymbol(String name, MontiCarTypeSymbolReference type) {
        MontiCarFieldSymbol formalParameterSymbol = new MontiCarFieldSymbol(name,
                MontiCarFieldSymbol.KIND, type);
        // init
        formalParameterSymbol.setParameter(true);
        return formalParameterSymbol;
    }

    @Override
    public MontiCarTypeSymbol createTypeVariable(String name) {
        MontiCarTypeSymbol typeVariableSymbol = new MontiCarTypeSymbol(name);
        // type variable init
        // TODO do these serve the same purpose? if yes type variable is redundant
        typeVariableSymbol.setFormalTypeParameter(true);
        return typeVariableSymbol;
    }
}
