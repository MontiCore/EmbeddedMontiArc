/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import de.rwth.montisim.commons.eventsimulation.EventTarget;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public abstract class EEEventProcessor implements EventTarget {
    public final transient EEComponentProperties properties;
	protected transient EESystem eesystem;
    public transient int id;
    
    public EEEventProcessor(EEComponentProperties properties){
        this.properties = properties;
    }

    public void attachTo(EESystem simulator) throws EEMessageTypeException {
        this.eesystem = simulator;
        this.id = simulator.getComponentManager().registerComponent(this, properties.priority);
        init();
    }

    protected abstract void init() throws EEMessageTypeException;

    @Override
    public String toString(){
        return "EEEventProcessor \""+properties.name+'"';
    }
}