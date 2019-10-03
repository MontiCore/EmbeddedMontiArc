/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes.AliasAttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes.LiteralAttributeSymbol;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes.EnvironmentAttributeSymbol;

import java.util.Optional;

public class AttributeSymbol extends AttributeSymbolTOP {
    public AttributeSymbol(String name) {
        super(name);
    }

    public boolean isAliasAttribute() {
        return this instanceof AliasAttributeSymbol;
    }

    public Optional<AliasAttributeSymbol> asAliasAttribute() {
        return this.isAliasAttribute() ? Optional.of((AliasAttributeSymbol) this) : Optional.empty();
    }

    public boolean isLiteralAttribute() {
        return this instanceof LiteralAttributeSymbol;
    }

    public Optional<LiteralAttributeSymbol> asLiteralAttribute() {
        return this.isLiteralAttribute() ? Optional.of((LiteralAttributeSymbol) this) : Optional.empty();
    }

    public boolean isEnvironmentAttribute() {
        return this instanceof EnvironmentAttributeSymbol;
    }

    public Optional<EnvironmentAttributeSymbol> asEnvironmentAttribute() {
        return this.isEnvironmentAttribute() ? Optional.of((EnvironmentAttributeSymbol) this) : Optional.empty();
    }
}
