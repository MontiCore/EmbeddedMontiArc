/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.components;

import java.util.Optional;

import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.EEDiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public abstract class EEEventProcessor {
    public final String name;
	protected EESimulator simulator;
    public int id;
    public Optional<Integer> priority;
    
    public EEEventProcessor(EEComponentProperties properties){
        this.name = properties.name;
        this.priority = properties.priority;
    }

    public void attachTo(EESimulator simulator) throws EEMessageTypeException {
        this.simulator = simulator;
        this.id = simulator.getComponentManager().registerComponent(this, priority);
        init();
    }

    protected abstract void init() throws EEMessageTypeException;
	public abstract EEComponentType getComponentType();
    public abstract void process(EEDiscreteEvent event);

    @Override
    public String toString(){
        return "EEEventProcessor \""+name+'"';
    }
}