/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import java.util.Optional;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.physicalvalue.PhysicalValueRegistry;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.message.MessagePriorityComparator;

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

    public static class ComponentBuildContext {
        public final PhysicalValueRegistry physicalValues;
        public final Updater componentUpdater;
        public final MessagePriorityComparator comp;
        public final Pathfinding pathfinding;

        public ComponentBuildContext(PhysicalValueRegistry physicalValues, Updater componentUpdater, MessagePriorityComparator msgComp, Pathfinding pathfinding){
            this.physicalValues = physicalValues;
            this.componentUpdater = componentUpdater;
            this.comp = msgComp;
            this.pathfinding = pathfinding;
        }
    }
    
    public abstract EEEventProcessor build(ComponentBuildContext context);
}