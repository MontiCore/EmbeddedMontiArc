/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.EEDiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public abstract class EEEventProcessor {
    public final transient EEComponentProperties properties;
	protected transient EESimulator simulator;
    public transient int id;
    
    public EEEventProcessor(EEComponentProperties properties){
        this.properties = properties;
    }

    public void attachTo(EESimulator simulator) throws EEMessageTypeException {
        this.simulator = simulator;
        this.id = simulator.getComponentManager().registerComponent(this, properties.priority);
        init();
    }

    protected abstract void init() throws EEMessageTypeException;
	public abstract EEComponentType getComponentType();
    public abstract void process(EEDiscreteEvent event);

    @Override
    public String toString(){
        return "EEEventProcessor \""+properties.name+'"';
    }
}