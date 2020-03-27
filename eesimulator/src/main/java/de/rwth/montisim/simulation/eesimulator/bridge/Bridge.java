/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.bridge;

import java.time.Duration;

import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.eesimulator.BusComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EEDiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;

public class Bridge extends BusComponent {

    /**
     * The delay that is added to each event if it is transmitted over this bride.
     */
    private final Duration delay;

    public Bridge(EESimulator simulator, String name, Duration delay) {
        super(simulator, name, 0);
        this.delay = delay;
    }

    public static Bridge newInstantBridge(EESimulator simulator, String name) {
        return new Bridge(simulator, name, Duration.ZERO);
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.BRIDGE;
    }

    @Override
    public void process(EEDiscreteEvent event) {
		switch(event.getEventType()){
			case MESSAGE_SEND:
				dispatchMessage((MessageSendEvent) event);
			break;
			case MESSAGE_RECEIVE:
                MessageReceiveEvent msgRecvEvent = (MessageReceiveEvent) event;
                if (this.delay.isZero()){
                    dispatchMessage(new MessageSendEvent(msgRecvEvent.getEventTime(), null, msgRecvEvent.getMessage()));
                } else {
                    this.simulator.addEvent(new MessageSendEvent(msgRecvEvent.getEventTime().plus(delay), this, msgRecvEvent.getMessage()));
                }
			break;
			default:
				throw new UnexpectedEventException(this.toString(), event);
		}
    }
}