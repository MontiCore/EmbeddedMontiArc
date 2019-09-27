/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import java.time.Instant;



public class KeepAliveEvent extends EEDiscreteEvent {
	
	public KeepAliveEvent(Instant eventTime, EEComponent target) {
		super(EEDiscreteEventTypeEnum.KEEP_ALIVE_EVENT, eventTime, target);
	}

}
