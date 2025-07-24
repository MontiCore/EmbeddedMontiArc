/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator;

import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public abstract class EEComponentProperties {
    public String name;
    public final Vector<String> connected_to = new Vector<>();
    public Optional<Integer> priority;
    // public boolean internal = false;

    public EEComponentProperties() {
        this.name = "UnnamedComponent"; // Will collide when building vehicle if multiple components are unnamed, => only a placeholder
        this.priority = Optional.empty();
    }

    public EEComponentProperties setPriority(int priority) {
        this.priority = Optional.of(priority);
        return this;
    }

    public EEComponentProperties setName(String name) {
        this.name = name;
        return this;
    }

    public EEComponentProperties connectTo(String componentName) {
        this.connected_to.add(componentName);
        return this;
    }

    public abstract EEComponentType getGeneralType();

    public abstract String getType();

    public boolean canTransferMessages() {
        return false;
    }

    // For components that can transfer messages: a search cost that represents the priority this component has to be used for routing
    public float routingCost() {
        return 0.0f;
    }

    public abstract EEComponent build(EESystem eesystem, BuildContext context) throws EEMessageTypeException;
}