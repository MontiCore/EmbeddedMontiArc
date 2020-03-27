/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.events;

import de.rwth.montisim.simulation.eesimulator.EEDiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.EEEventType;
import java.time.Instant;

public class ExecuteEvent extends EEDiscreteEvent {
	
	public ExecuteEvent(Instant eventTime, EEEventProcessor target) {
		super(eventTime, target);
    }

	@Override
	public EEEventType getEventType(){
		return EEEventType.EXECUTE;
	}
}
