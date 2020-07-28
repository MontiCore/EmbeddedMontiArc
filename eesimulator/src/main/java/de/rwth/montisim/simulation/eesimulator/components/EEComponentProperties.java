/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import java.util.Optional;

public abstract class EEComponentProperties {
    public String name;
    public Optional<Integer> priority;
    // public boolean internal = false;

    public EEComponentProperties() {
        this.name = "UnnamedComponent"; // Will collide when building vehicle if multiple components are unnamed, => only a placeholder
        this.priority = Optional.empty();
    }

    public EEComponentProperties setPriority(int priority){
        this.priority = Optional.of(priority);
        return this;
    }

    public EEComponentProperties setName(String name){
        this.name = name;
        return this;
    }

    public abstract EEComponentType getGeneralType();
    public abstract String getType();

    // public EEComponentProperties setInternal(){
    //     this.internal = true;
    //     return this;
    // }

}