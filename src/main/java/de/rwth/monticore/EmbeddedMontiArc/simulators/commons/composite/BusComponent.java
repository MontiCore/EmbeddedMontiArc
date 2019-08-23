/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.composite;

/**
   A BusComponent structure represents a tree-like data-set using the Composite Pattern.
   The elements can be BusValue, BusComposite or BusArray.
   getType() will return the type of the Component.
 */
public interface BusComponent {
    enum ComponentType {
        VALUE,
        ARRAY,
        COMPOSITE
    }
    ComponentType getType();
}
