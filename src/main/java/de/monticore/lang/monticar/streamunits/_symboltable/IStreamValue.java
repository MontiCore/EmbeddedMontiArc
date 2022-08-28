/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

import de.monticore.lang.monticar.streamunits._symboltable.NamedStreamUnitsSymbol;

/**
 */
public interface IStreamValue {
    default boolean isStreamValuePrecision() {
        return false;
    }

    default boolean isStreamValueDontCare() {
        return false;
    }

    default boolean isStreamValueAtTick() {
        return false;
    }

    void visit(NamedStreamUnitsSymbol streamUnitsSymbol);

    String toString();
}
