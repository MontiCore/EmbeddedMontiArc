/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator;

import java.lang.reflect.InvocationTargetException;
import java.util.Optional;

import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.CustomJson;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ValueType;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationContext;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.events.EEDiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.events.EEEventType;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.EEDiscreteEvent.EventData;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupErrors;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessagePriorityComparator;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;

/**
 * Discrete event simulator that takes care of the communication in the car.
 * 
 * EE Setup Process: - Create an EESimulator. - Create buses -> Give
 * EESimulator. - Create Components/Bridges -> Give EESimulator. -
 * connectToBus(id/name/bus) for components/bridges. - finalizeSetup()
 * 
 * For more information on the EE-system, see [docs/eesimulator.md]
 */
public class EESimulator extends DiscreteEventSimulator<EEEventType, EEDiscreteEvent> implements CustomJson {

	// Vector<DataType> types = new Vector<>();
	// HashMap<DataType, Integer> typeIds = new HashMap<>();

	protected final MessagePriorityComparator msgPrioComp;
	protected final ComponentManager componentManager;
	protected final MessageTypeManager messageTypeManager;
	protected final EESetupErrors errors;
	private boolean finalized = false;

	public EESimulator(MessageTypeManager messageTypeManager) {
		this.errors = new EESetupErrors();
		this.componentManager = new ComponentManager(this.errors);
		this.messageTypeManager = messageTypeManager;
		this.msgPrioComp = new MessagePriorityComparator(messageTypeManager, componentManager);
	}

	public void processEvent(EEDiscreteEvent event) {
		event.getTarget().process(event);
	}

	public MessageTypeManager getMessageTypeManager() {
		return messageTypeManager;
	}

	public ComponentManager getComponentManager() {
		return componentManager;
	}

	public MessagePriorityComparator getMsgPrioComp() {
		return msgPrioComp;
	}

	public void finalizeSetup() throws EESetupException {
		componentManager.finalizeSetup();
		errors.throwExceptions();
		this.finalized = true;
	}

	public boolean isFinalized() {
		return finalized;
	}

	public static final String K_COMPONENTS = "components";
	public static final String K_EVENTS = "events";

	public static class EESerializationContext implements SerializationContext {
		public final ComponentManager cm;
		public final MessageTypeManager mtm;
		public EESerializationContext(ComponentManager cm, MessageTypeManager mtm){
			this.cm = cm;
			this.mtm = mtm;
		}
	}

	@Override
	public void write(JsonWriter w, SerializationContext context) throws IllegalAccessException {
		context = new EESerializationContext(componentManager, messageTypeManager);
		w.startObject();
		w.writeKey(K_COMPONENTS);
		w.startObject();
		// Save all component states (name: {component state})
		for (EEEventProcessor e : componentManager.componentTable) {
			w.writeKey(e.properties.name);
			Json.toJson(w, e, context);
		}
		w.endObject();
		w.writeKey(K_EVENTS);
		w.startArray();
		// Save all events -> resolve ids
		for (EEDiscreteEvent e : eventList) {
			if (e instanceof MessageReceiveEvent && ((MessageReceiveEvent)e).invalid) continue;
			Json.toJson(w, e.getEventData(), context);
		}
		w.endArray();
		w.endObject();
	}

	@Override
	public void read(JsonTraverser t, ObjectIterable it, SerializationContext context)
			throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException {
		context = new EESerializationContext(componentManager, messageTypeManager);
		for (Entry e : t.streamObject()) {
			if (e.key.equals(K_COMPONENTS)) {
				for (Entry e2 : t.streamObject()) {
					String name = e2.key.getJsonString();
					Optional<EEEventProcessor> o = componentManager.getComponent(name);
					if (!o.isPresent())
						throw new ParsingException("Unknown component: " + name);
					Json.fromJson(t, o.get(), context);
				}
			} else if (e.key.equals(K_COMPONENTS)) {
				for (ValueType vt : t.streamArray()) {
					eventList.offer(Json.instantiateFromJson(t, EventData.class, context).getEvent(componentManager));
				}
			} else
				t.unexpected(e);
		}
	}
}
