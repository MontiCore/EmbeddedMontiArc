/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import java.time.Instant;
import java.util.UUID;


public abstract class EEDiscreteEvent {

    private final Instant eventTime;

    private final EEComponent target;
    
    private final EEDiscreteEventTypeEnum eventType;
    
    private final UUID Id;

    public EEDiscreteEvent(EEDiscreteEventTypeEnum eventType, Instant eventTime, EEComponent target){
        this.eventTime = eventTime;
        this.target = target;
        this.eventType = eventType;
        this.Id = UUID.randomUUID();
    }

    public Instant getEventTime() {
        return eventTime;
    }

    public EEComponent getTarget() {
        return target;
    }
    
    public EEDiscreteEventTypeEnum getEventType() {
    	return this.eventType;
    }
    
    public UUID getId() {
    	return this.Id;
    }
    
    @Override
    public String toString() {
    	return "Type = " + this.eventType +
    			"; Target  = " + this.target +
    			"; Event Time = " +this.eventTime +
    			"; Id = " + this.Id;
    			
    }
}
