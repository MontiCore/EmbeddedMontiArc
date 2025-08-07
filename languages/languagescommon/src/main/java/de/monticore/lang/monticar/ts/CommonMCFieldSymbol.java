/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;

/**
 */
public abstract class CommonMCFieldSymbol<T extends MCTypeReference<? extends MCTypeSymbol>> extends CommonSymbol implements MCFieldSymbol {

    private T type;

    private boolean isParameter = false;

    public CommonMCFieldSymbol(String name) {
        this(name, MCFieldSymbol.KIND);
    }

    public CommonMCFieldSymbol(String name, MCAttributeSymbolKind kind) {
        super(name, kind);
    }

    @Override
    public T getType() {
        return type;
    }

    public void setType(T type) {
        this.type = type;
    }

    @Override
    public boolean isParameter() {
        return isParameter;
    }

    public void setParameter(boolean isParameter) {
        this.isParameter = isParameter;
    }
}
