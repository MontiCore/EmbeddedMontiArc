/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts.references;

import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.symboltable.Scope;

public class MontiCarTypeSymbolReference extends CommonMCTypeReference<MontiCarTypeSymbol> implements MCTypeReference<MontiCarTypeSymbol> {

    public MontiCarTypeSymbolReference(String name, Scope definingScopeOfReference, int arrayDimension) {
        super(name, MontiCarTypeSymbol.KIND, definingScopeOfReference);
        setDimension(arrayDimension);
    }
}
