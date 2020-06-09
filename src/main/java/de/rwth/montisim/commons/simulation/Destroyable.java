/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

/** Implemented by Simulation objects that need special destruction code (ex: Native Bound code cleanup) */
public interface Destroyable {
    void destroy();
}
