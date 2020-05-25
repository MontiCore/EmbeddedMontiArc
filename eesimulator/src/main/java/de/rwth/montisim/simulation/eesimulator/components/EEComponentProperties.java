/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.components;

import java.util.Optional;
import java.util.Vector;

public abstract class EEComponentProperties {
    public final EEComponentType componentType;
    public String name;
    public Optional<Integer> priority;
    public final Vector<String> buses = new Vector<>();

    public EEComponentProperties(EEComponentType componentType) {
        this.componentType = componentType;
        this.name = "UnnamedComponent";
        this.priority = Optional.empty();
        buses.add("DefaultBus");
    }
}