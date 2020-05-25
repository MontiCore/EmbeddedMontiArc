/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.bridge;

import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.eesimulator.components.BusComponent;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.events.EEDiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;

public class Bridge extends BusComponent {

    public final BridgeProperties properties;

    public Bridge(BridgeProperties properties) {
        super(properties);
        this.properties = properties;
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
                if (properties.delay.isZero()){
                    dispatchMessage(new MessageSendEvent(msgRecvEvent.getEventTime(), null, msgRecvEvent.getMessage()));
                } else {
                    this.simulator.addEvent(new MessageSendEvent(msgRecvEvent.getEventTime().plus(properties.delay), this, msgRecvEvent.getMessage()));
                }
			break;
			default:
				throw new UnexpectedEventException(this.toString(), event);
		}
    }

    @Override
    protected void init() {
        
    }
}