/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator;

import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Vector;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.eventsimulation.*;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.commons.utils.json.JsonTraverser.*;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.events.*;
import de.rwth.montisim.simulation.eesimulator.events.EEEvent.EventData;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.message.*;
import de.rwth.montisim.simulation.eesimulator.sensor.Sensor;

/**
 * Handles the EEComponents of the vehicle and their setup.
 * <p>
 * For more information on the EE-system, see [docs/eesimulator.md]
 */
public class EESystem implements CustomJson, BuildObject {
    public static final String CONTEXT_KEY = "eesystem";
    public static final String COMPONENT_POPPER_CONTEXT_KEY = "component_popper";
    public static final String COMPONENT_UPDATER_CONTEXT_KEY = "component_updater";
    public static final String COMPONENT_DESTROYER_CONTEXT_KEY = "component_destroyer";
    //public static final String START_TIME_CONTEXT_KEY = "start_time";

    // Vector<DataType> types = new Vector<>();
    // HashMap<DataType, Integer> typeIds = new HashMap<>();
    public final DiscreteEventSimulator simulator;
    protected final MessagePriorityComparator msgPrioComp;


    /**
     * All registered Components (Buses, Bridges & EEComponents)
     */
    public final Vector<EEComponent> componentTable = new Vector<>();
    private final Vector<Integer> componentPriority = new Vector<>();
    private final HashMap<String, EEComponent> componentsByName = new HashMap<>();
    /** All "Client" Components (non-bus & non-bridge) */
    //public final List<EEComponent> components = new Vector<>();
    /** All registered Bridges */
    //public final List<Bridge> bridges = new Vector<>();
    /**
     * All registered Buses
     */
    //public final List<Bus> buses = new Vector<>();
    private final List<PortTagUser> tagUsers = new Vector<>();

    private final HashMap<String, Integer> msgIdMap = new HashMap<>();
    private final Vector<DataType> msgTypes = new Vector<>();
    private int msgIdCounter = 0;

    protected final EESetupErrors errors;

    private boolean finalized = false;

    public boolean isFinalized() {
        return finalized;
    }


    public EESystem(DiscreteEventSimulator simulator) {
        this.simulator = simulator;
        this.errors = new EESetupErrors();
        this.msgPrioComp = new MessagePriorityComparator(this);
    }

    public MessagePriorityComparator getMsgPrioComp() {
        return msgPrioComp;
    }

    public int getMessageId(String name, DataType msgType) {
        Integer i = msgIdMap.get(name);
        if (i != null) return i;
        int newId = msgIdCounter;
        msgIdCounter++;
        msgIdMap.put(name, newId);
        msgTypes.add(msgType);
        return newId;
    }

    public DataType getMsgType(int msgId) {
        return msgTypes.elementAt(msgId);
    }

    public DataType getMsgType(String name) {
        return msgTypes.elementAt(msgIdMap.get(name));
    }


    public Optional<Actuator> getActuator(String name) {
        EEComponent e = componentsByName.get(name);
        if (e == null || e.properties.getGeneralType() != EEComponentType.ACTUATOR) return Optional.empty();
        return Optional.of((Actuator) e);
    }

    public Optional<Sensor> getSensor(String name) {
        EEComponent e = componentsByName.get(name);
        if (e == null || e.properties.getGeneralType() != EEComponentType.SENSOR) return Optional.empty();
        return Optional.of((Sensor) e);
    }

    public Optional<Bridge> getBridge(String name) {
        EEComponent e = componentsByName.get(name);
        if (e == null || e.properties.getGeneralType() != EEComponentType.BRIDGE) return Optional.empty();
        return Optional.of((Bridge) e);
    }

    public Optional<EEComponent> getComponent(String name) {
        EEComponent e = componentsByName.get(name);
        if (e == null) return Optional.empty();
        return Optional.of(e);
    }

    /**
     * NOTE: the component's name must be set before registering it.
     */
    public int registerComponent(EEComponent comp, Optional<Integer> priority) {
        if (componentsByName.containsKey(comp.properties.name)) {
            errors.namesErrors.add(new EEComponentNameException(comp.properties.name));
        }
        int id = componentTable.size();
        componentTable.add(comp);
        componentPriority.add(priority.isPresent() ? priority.get() : 0);
        componentsByName.put(comp.properties.name, comp);
        // if (comp instanceof Bridge) {
        //     bridges.add((Bridge) comp);
        // } else if (comp instanceof Bus) {
        //     buses.add((Bus) comp);
        // } else {
        //     components.add(comp);
        // }
        if (comp instanceof PortTagUser) {
            tagUsers.add((PortTagUser) comp);
        }
        return id;
    }


    public int getComponentPriority(int componentId) {
        return componentPriority.get(componentId);
    }

    /**
     * Sets the priority of all <b>registered</b> components contained in the
     * priorities list. The higher the number, the higher the priority.
     */
    public void addComponentPriorities(List<Pair<String, Integer>> priorities) {
        for (Pair<String, Integer> p : priorities) {
            if (componentsByName.containsKey(p.getKey())) {
                componentPriority.set(componentsByName.get(p.getKey()).id, p.getValue());
            }
        }
    }

    /**
     * Sets the priority of all <b>registered</b> messages contained in the priorities list.
     * The lower the number, the higher the priority.
     */
    public void addMessagePriorities(List<Pair<String, Integer>> priorities) {
        for (EEComponent comp : componentTable) {
            for (Pair<String, Integer> p : priorities) {
                MessageInformation info = comp.getMsgInfo(p.getKey());
                if (info != null) info.priority = p.getValue();
            }
        }
    }


    public void finalizeSetup() throws EESetupException {
        resolveTags();
        checkMessageTypes();
        computeMessageRouting();
        errors.throwExceptions();
        this.finalized = true;
    }

    private void resolveTags() {
        if (tagUsers.size() == 0) return;
        for (EEComponent c : componentTable) {
            for (PortInformation pi : c.ports) {
                for (String tag : pi.tags) {
                    for (PortTagUser tu : tagUsers) {
                        if (tu.getUsedTags().contains(tag)) {
                            tu.processTag(tag, pi);
                        }
                    }
                }
            }
        }
    }

    private void checkMessageTypes() {
        HashMap<String, List<Pair<EEComponent, DataType>>> typeMap = new HashMap<>();
        for (EEComponent c : componentTable) {
            for (PortInformation pi : c.ports) {
                List<Pair<EEComponent, DataType>> compList = typeMap.get(pi.name);
                if (compList == null) {
                    compList = new Vector<>();
                    typeMap.put(pi.name, compList);
                }
                compList.add(new Pair<>(c, pi.data_type));
            }
        }

        for (java.util.Map.Entry<String, List<Pair<EEComponent, DataType>>> e : typeMap.entrySet()) {
            String msgName = e.getKey();
            List<Pair<EEComponent, DataType>> compList = e.getValue();
            DataType firstType = compList.get(0).getValue();
            for (int i = 1; i < compList.size(); ++i) {
                if (!compList.get(i).getValue().equals(firstType)) {
                    errors.msgTypeExceptions.add(new EEMessageTypeException(msgName, compList));
                    break;
                }
            }
        }
    }


    static class CostComparator implements Comparator<Pair<Integer, Integer>> {
        final float costs[];

        public CostComparator(float costs[]) {
            this.costs = costs;
        }

        public int compare(Pair<Integer, Integer> a, Pair<Integer, Integer> b) {
            return costs[a.getKey()] < costs[b.getKey()] ? -1 : 1;
        }
    }

    private void computeMessageRouting() {
        int compCount = componentTable.size();
        boolean visited[] = new boolean[compCount];
        float nodeCosts[] = new float[compCount];
        int predecessor[] = new int[compCount];
        HashMap<String, PortInformation> ports[] = new HashMap[compCount];
        // For a given component: each entry in the table lists the following nodes a given message has to follow
        HashMap<MessageInformation, Set<EEComponent>> routings[] = new HashMap[compCount];
        HashMap<String, Set<EEComponent>> inputMap[] = new HashMap[compCount];
        for (EEComponent component : componentTable) {
            HashMap<String, PortInformation> map = new HashMap<>();
            for (PortInformation p : component.ports) map.put(p.name, p);
            ports[component.id] = map;

            routings[component.id] = new HashMap<>();
            inputMap[component.id] = new HashMap<>();
        }
        // Pair: <next node, predecessor node>
        PriorityQueue<Pair<Integer, Integer>> nextNodes = new PriorityQueue<>(new CostComparator(nodeCosts));

        for (EEComponent component : componentTable) {
            Arrays.fill(visited, false);
            Arrays.fill(nodeCosts, Float.POSITIVE_INFINITY);
            Arrays.fill(predecessor, -2);

            // Breadth first traversal starting from the component
            nodeCosts[component.id] = 0;
            nextNodes.add(new Pair<>(component.id, -1));

            while (!nextNodes.isEmpty()) {
                Pair<Integer, Integer> next = nextNodes.poll();
                int next_id = next.getKey();
                if (visited[next_id]) continue;
                visited[next_id] = true;
                predecessor[next_id] = next.getValue();
                EEComponent nextComponent = componentTable.elementAt(next_id);

                // Check inputs
                if (next_id != component.id) {
                    HashMap<String, PortInformation> portMap = ports[next_id];
                    component.streamOutputPorts().forEach(p -> {
                        PortInformation otherPort = portMap.get(p.name);
                        if (otherPort != null && otherPort.isInput()) {
                            // Found
                            backtrack(component.msgInfos.get(p.name), next_id, predecessor, routings);
                            // Add to inputs
                            Set<EEComponent> senders = inputMap[next_id].get(p.name);
                            if (senders == null) {
                                senders = new HashSet<>();
                                inputMap[next_id].put(p.name, senders);
                            }
                            senders.add(component);
                        }
                    });
                }

                // Check neighbors if start component or 'canTransfer' component
                if (next_id != component.id && !nextComponent.properties.canTransferMessages()) continue;
                float newCost = nodeCosts[next_id] + nextComponent.properties.routingCost();
                for (EEComponent neighbor : nextComponent.connectedComponents) {
                    if (visited[neighbor.id]) continue;
                    nodeCosts[neighbor.id] = newCost;
                    nextNodes.add(new Pair<>(neighbor.id, next_id));
                }
            }
        }

        // Set final routing table 
        for (EEComponent component : componentTable) {
            for (java.util.Map.Entry<MessageInformation, Set<EEComponent>> e : routings[component.id].entrySet()) {
                Vector<EEComponent> vec = new Vector<>();
                for (EEComponent c : e.getValue()) vec.add(c);
                component.msgRoutingTable.put(e.getKey(), vec);
            }
        }

        // Check multiple inputs allowed, optional/required
        for (EEComponent component : componentTable) {
            component.streamInputPorts().forEach(p -> {
                int count = 0;
                Set<EEComponent> senders = inputMap[component.id].get(p.name);
                if (senders != null) count = senders.size();
                if (!p.optional && count < 1)
                    errors.missingOutputExceptions.add(new EEMissingOutputException(p.name, component.properties.name));
                if (!p.allows_multiple_inputs && count > 1)
                    errors.multipleInputsExceptions.add(new EEMultipleInputsException(component, p.name, senders));
            });
        }
    }

    private void backtrack(MessageInformation msgInfo, int node, int predecessor[], HashMap<MessageInformation, Set<EEComponent>> routings[]) {
        EEComponent nextComponent = componentTable.elementAt(node);
        int prevNode = predecessor[node];
        while (prevNode >= 0) {
            Set<EEComponent> nextTargets = routings[prevNode].get(msgInfo);
            if (nextTargets == null) {
                nextTargets = new HashSet<>();
                routings[prevNode].put(msgInfo, nextTargets);
            }
            if (!nextTargets.contains(nextComponent)) {
                nextTargets.add(nextComponent);
            } else break; // Stop backtracking: the given msginfo already reaches the next node

            nextComponent = componentTable.elementAt(prevNode);
            prevNode = predecessor[prevNode];
            if (prevNode < -1) throw new IllegalStateException("Reached invalid predecessor.");
        }
    }


    public static final String K_COMPONENTS = "components";
    public static final String K_EVENTS = "events";


    @Override
    public void write(JsonWriter w, BuildContext context) throws SerializationException {
        BuildContext subcontext = new BuildContext(context);
        subcontext.addObject(this);
        w.startObject();
        w.writeKey(K_COMPONENTS);
        w.startObject();
        // Save all component states (name: {component state})
        for (EEComponent e : componentTable) {
            w.writeKey(e.properties.name);
            Json.toJson(w, e, subcontext);
        }
        w.endObject();
        w.writeKey(K_EVENTS);
        w.startArray();
        // Save all events -> resolve ids
        for (DiscreteEvent e : simulator.getEventList()) {
            if (e.invalid || !(e instanceof EEEvent)) continue;
            if (!(e.getTarget() instanceof EEComponent)) continue;
            if (((EEComponent) e.getTarget()).eesystem != this) continue; // Event for another vehicle
            Json.toJson(w, ((EEEvent) e).getEventData(), subcontext);
            e.invalid = true; // Remove from the event simulator
        }
        w.endArray();
        w.endObject();
    }

    @Override
    public void read(JsonTraverser t, ObjectIterable it, BuildContext context) throws SerializationException {
        BuildContext subcontext = new BuildContext(context);
        subcontext.addObject(this);
        for (Entry e : t.streamObject()) {
            if (e.key.equals(K_COMPONENTS)) {
                for (Entry e2 : t.streamObject()) {
                    String name = e2.key.getJsonString();
                    Optional<EEComponent> o = getComponent(name);
                    if (!o.isPresent())
                        throw new ParsingException("Unknown component: " + name);
                    Json.fromJson(t, o.get(), subcontext);
                }
            } else if (e.key.equals(K_EVENTS)) {
                for (ValueType vt : t.streamArray()) {
                    simulator.getEventList().offer(Json.instantiateFromJson(t, EventData.class, subcontext).getEvent(this));
                }
            } else
                t.unexpected(e);
        }
    }

    @Override
    public String getKey() {
        return CONTEXT_KEY;
    }
}
