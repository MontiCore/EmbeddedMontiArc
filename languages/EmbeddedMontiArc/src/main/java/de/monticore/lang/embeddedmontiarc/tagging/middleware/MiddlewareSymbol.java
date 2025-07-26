/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tagging.middleware;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

public abstract class MiddlewareSymbol extends TagSymbol {
    public MiddlewareSymbol(TagKind kind, Object... values) {
        super(kind, values);
    }
}
