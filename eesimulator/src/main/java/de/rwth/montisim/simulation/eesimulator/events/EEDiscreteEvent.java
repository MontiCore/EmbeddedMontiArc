/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.events;

import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

import java.time.Instant;

/**
 * Discrete Events that are processed by the EESimulator.
 */
public abstract class EEDiscreteEvent extends DiscreteEvent<EEEventType> {
    private final EEEventProcessor target;

    public EEDiscreteEvent(Instant eventTime, EEEventProcessor target){
        super(eventTime);
        this.target = target;
    }

    public EEEventProcessor getTarget(){
        return target;
    }
    
    @Override
    public String toString() {
    	return "EEDiscreteEvent {type: " + getEventType() +
    			", target: " + this.target +
    			", time: " + this.time + "}";
    }
}