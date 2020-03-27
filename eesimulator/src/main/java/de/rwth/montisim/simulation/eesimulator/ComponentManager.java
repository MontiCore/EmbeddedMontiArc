/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator;

import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Vector;

// import org.jfree.util.Log;

import de.rwth.montisim.commons.utils.Graph;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.message.PortInformation;

/**
 * Gathers all EEComponents (and Buses) of an EEVehicle, Manages their IDs And
 * performs routing pre-computations (assign targets, ...)
 */
public class ComponentManager {
    private final Vector<EEEventProcessor> componentTable = new Vector<>();
    private final Vector<Integer> componentPriority = new Vector<>();
    private final HashMap<String, EEEventProcessor> componentsByName = new HashMap<>();
    // Non-Bridge and Non-Bus components
    private final List<EEComponent> components = new ArrayList<>();
    private final List<Bridge> bridges = new ArrayList<>();
    private final List<Bus> buses = new ArrayList<>();
    private final EESetupErrors errors;

    public ComponentManager(EESetupErrors errors){
        this.errors = errors;
    }

    /**
     * NOTE: the component's name must be set before registering it.
     */
    public int registerComponent(EEEventProcessor comp, int priority) {
        if (componentsByName.containsKey(comp.name)){
            errors.namesErrors.add(new EEComponentNameException(comp.name));
        }
        int id = componentTable.size();
        componentTable.add(comp);
        componentPriority.add(priority);
        componentsByName.put(comp.name, comp);
        if (comp instanceof Bridge) {
            bridges.add((Bridge) comp);
        } else if (comp instanceof EEComponent) {
            components.add((EEComponent) comp);
        } else if (comp instanceof Bus) {
            buses.add((Bus) comp);
        } else
            throw new IllegalArgumentException("Unknown EEEventProcessor type: " + comp);
        return id;
    }

    public int registerComponent(EEEventProcessor comp) {
        return registerComponent(comp, 0);
    }

    public Bus getBus(String name) {
        EEEventProcessor p = componentsByName.get(name);
        if (p == null){
            errors.missingComponentExceptions.add(new EEMissingComponentException(name));
            return null;
        }
        if (!(p instanceof Bus)){
            errors.componentTypeExceptions.add(new EEComponentTypeException(name, p.getComponentType(), EEComponentType.BUS));
            return null;
        }
        return (Bus) p;
    }

    public Bus getBus(int componentId) {
        if (componentId < 0 || componentId >= componentTable.size()){
            errors.invalidIdExceptions.add(new EEInvalidComponentIdException(componentId, componentTable.size()));
            return null;
        }
        EEEventProcessor p = componentTable.get(componentId);
        if (!(p instanceof Bus)){
            errors.componentTypeExceptions.add(new EEComponentTypeException(p.name, p.getComponentType(), EEComponentType.BUS));
            return null;
        }
        return (Bus) p;
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

    // Currently: minimizes traffic
    public void finalizeSetup() {
        if (componentTable.size() == 0) return;
		// Create EE system graph.
		// - Vertices: Components, Buses, Bridges (with type)
		// - Edges: Components are connected together
        Graph graph = new Graph(componentTable.size());
        Vector<Integer> type = graph.newColor(0); // 0 for bus, 1 for bridge, 2 for components
        // Use the component id as vertex id
        for (int i = 0; i < componentTable.size(); ++i){
            EEEventProcessor e = componentTable.get(i);
            type.set(i, e.getComponentType() == EEComponentType.BUS ? 0 : e.getComponentType() == EEComponentType.BRIDGE ? 1 : 2);
        }
        for (Bus b : buses){
            for (BusComponent c : b.getConnectedComponents()){
                graph.addUndirectedEdge(b.id, c.id);
            }
        }

        // Verify no cyclic bus connections. (Temporary?)
        // - DFS, mark nodes
        // - If encountering already marked node that is not the current parent -> found cycle
        Vector<Boolean> visited = graph.newColor(false);
        Vector<Integer> parent = graph.newColor(-1);
        Deque<Integer> stack = graph.newStack();

        stack.push(0);
        visited.set(0, true);

        while (stack.size() > 0){
            int next = stack.pop();
            int p = parent.get(next);
            for (int a : graph.adjacencies.get(next)){
                if (visited.get(a)){
                    if (a != p) {
                        errors.cyclicError = Optional.of(new EECyclicSetupException());
                        return;
                    }
                } else {
                    parent.set(a, next);
                    visited.set(a, true);
                    stack.push(a);
                }
            }
        }

		// Propagate outputs:
        
		// - For all INPUTS: boolean tables: flag if message is sent by someone
        Vector<HashMap<String, Boolean>> inputsCovered = graph.<HashMap<String, Boolean>>newColor(null);
        for (EEComponent ec : components){
            List<PortInformation> ports = ec.getInputPorts();
            if (ports.size() > 0){
                HashMap<String, Boolean> map = new HashMap<>();
                inputsCovered.set(ec.id, map);
                for (PortInformation p : ports){
                    map.put(p.msg.name, false);
                }
            }
        }
        

        Vector<Boolean> usesOutput = graph.newColor(false);
        // - Per output port:
        for (EEComponent ec : components){
            List<PortInformation> ports = ec.getOutputPorts();
            for (PortInformation p : ports){
                //	 - Propagate reachable:
                //     - Error if same output
                //	   - Mark input table
                //	   - Color graph if used
                //     - Mark "backpointer" in graph -> "parent" color, also used to traverse the graph (only once)

                for (int j = 0; j < usesOutput.size(); ++j){
                    usesOutput.set(j, false);
                }
                for (int j = 0; j < parent.size(); ++j){
                    parent.set(j, -1);
                }

                stack.push(ec.id);

                while (stack.size() > 0){
                    int next = stack.pop();
                    int par = parent.get(next);
                    for (int a : graph.adjacencies.get(next)){
                        if (a != par){
                            parent.set(a, next);

                            if (type.get(a) == 2){
                                List<PortInformation> outputs = ((EEComponent) componentTable.get(a)).getOutputPorts();
                                for (PortInformation o : outputs) 
                                    if (o.msg.name.equals(p.msg.name))
                                        errors.addOutputOverlap(p.msg.name, new EEOutputOverlapException(ec.name));
                                HashMap<String, Boolean> map = inputsCovered.get(a);
                                if (map != null && map.containsKey(p.msg.name)){
                                    map.put(p.msg.name, true);
                                    usesOutput.set(a, true);
                                }
                            } else {
                                stack.push(a); // Only propagate through Buses and Bridges
                            }
                        }
                    }
                }

                for (int j = 0; j < visited.size(); ++j){
                    visited.set(j, false);
                }
                for (int j = 0; j < componentTable.size(); ++j){
                    if (usesOutput.get(j)){
                        stack.push(j);
                        visited.set(j, true);
                    }
                }
                //   - Warning if not used
                if (stack.size() == 0){
                    //TODO adapt
                    //Log.warn("Output: "+p.msg.name + " of component "+ec.name+ " not used.");
                }

                //   - Propagate "Usage" color (following backpointers)
                while (stack.size() > 0){
                    int next = stack.pop();
                    int par = parent.get(next);
                    for (int a : graph.adjacencies.get(next)){
                        if (a == par && !visited.get(a)){
                            visited.set(a, true);
                            usesOutput.set(a, true);
                            stack.push(a);
                        }
                    }
                }

                //   - Add routing info to Buses, Bridges and the message sender (collect neighbors marked with "usage" & different from backpointer).
                for (int j = 0; j < componentTable.size(); ++j){
                    if (usesOutput.get(j)){
                        int t = type.get(j);
                        if (ec.id == j || t == 1){
                            List<Bus> targets = new ArrayList<>();
                            for (int n : graph.adjacencies.get(j)){
                                if (usesOutput.get(n) && n != parent.get(j)){
                                    if (type.get(n) != 0) throw new IllegalStateException("Incorrect neighboring.");
                                    targets.add(((Bus) componentTable.get(n)));
                                }
                            }
                            ((BusComponent) componentTable.get(j)).addMessageTargets(p.msg.messageId, targets);
                        } else if (t == 0){
                            List<BusComponent> targets = new ArrayList<>();
                            for (int n : graph.adjacencies.get(j)){
                                if (usesOutput.get(n) && n != parent.get(j)){
                                    if (type.get(n) == 0) throw new IllegalStateException("Incorrect neighboring.");
                                    targets.add(((BusComponent) componentTable.get(n)));
                                }
                            }
                            ((Bus) componentTable.get(j)).addMessageTargets(p.msg.messageId, targets);
                        }
                    }
                }

                // If message not used -> Set empty targets @ Component
                if (!visited.get(ec.id)){
                    ec.addMessageTargets(p.msg.messageId, new ArrayList<>());
                }
            }
        }
        
        // Check input tables: error if non-optional not covered.
        int i = -1;
        for (HashMap<String,Boolean> map : inputsCovered){
            ++i;
            if (map == null) continue;
            EEComponent c = ((EEComponent) componentTable.get(i));
            for (PortInformation p : c.getInputPorts()){
                if (!map.get(p.msg.name) && !p.optional){
                    errors.missingOutputExceptions.add(new EEMissingOutputException(p.msg.name, c.name));
                }
            }
        }
    }
}