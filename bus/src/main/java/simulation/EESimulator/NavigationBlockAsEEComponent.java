/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.NavigationEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.Vertex;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.IAdjacency;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.IControllerNode;
import de.rwth.monticore.EmbeddedMontiArc.simulators.controller.navigation.navigationBlock.NavigationBlock;
import de.topobyte.osm4j.core.model.iface.OsmNode;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.bus.Bus;
import simulation.bus.BusMessage;
import simulation.environment.WorldModel;
import simulation.environment.osm.IntersectionFinder;
import simulation.util.Log;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

public class NavigationBlockAsEEComponent extends ImmutableEEComponent {
    private List<Vertex> trajectory;
    private Optional<IControllerNode> lastNavigationTarget;
    private Instant lastUpdate;

    private final NavigationBlock functionBlock;
    private final Map<BusEntry, Object> externalInputs;

    public static NavigationBlockAsEEComponent createNavigationBlockAsEEComponent(Bus bus) {
        return createNavigationBlockAsEEComponent(Collections.singletonList(bus));
    }

    public static NavigationBlockAsEEComponent createNavigationBlockAsEEComponent(List<Bus> buses){
        List<BusEntry> subscribedMessages = new ArrayList<>();
        subscribedMessages.add(BusEntry.CONSTANT_WHEELBASE);
        subscribedMessages.add(BusEntry.SENSOR_GPS_COORDINATES);
        List<EEComponent> targets = new ArrayList<EEComponent>(buses);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        targetsByMessageId.put(BusEntry.NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, targets);
        return new NavigationBlockAsEEComponent(buses.get(0).getSimulator(), subscribedMessages, targetsByMessageId);

    }

    public NavigationBlockAsEEComponent(EESimulator simulator, List<BusEntry> subscribedMessages, HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(simulator, EEComponentType.NAVIGATION, subscribedMessages, targetsByMessageId);
        this.functionBlock = new NavigationBlock();
        this.externalInputs = new HashMap<BusEntry, Object>();
        this.lastNavigationTarget = Optional.empty();
        this.lastUpdate = this.getSimulator().getSimulationTime();
    }

    @Override
    public void processEvent(EEDiscreteEvent event) {
        if(event.getEventType() != EEDiscreteEventTypeEnum.BUSMESSAGE){
            throw new IllegalArgumentException("NavigationBlockAsEEComponent expects BusMessage as event. Event type was" + event.getEventType());
        }
        BusMessage busMessage = (BusMessage)event;
        if(!this.getSubscribedMessages().contains(busMessage.getMessageID())){
            throw new IllegalArgumentException("NavigationBlockAsEEComponent got message with invalid message id. Message id was" + event.getEventType());
        }
        externalInputs.put(busMessage.getMessageID(), busMessage.getMessage());
    }

    /**
     * Function that initiates or updates navigation of the vehicle to a specified point in the map
     * Controller is periodically called such that setting these values in the function here should work without issues
     *
     * @param node Target node for navigation
     */
    public void navigateTo(IControllerNode node) {
        navigateTo(node, Collections.synchronizedList(new LinkedList<RealVector>()));
    }

    /**
     * Function that initiates or updates navigation of the vehicle to a specified point in the map
     * Controller is periodically called such that setting these values in the function here should work without issues
     * Tries to avoid list of coordinates, might not be possible if all ways to target are affected. Then avoiding coordinates is not possible.
     *
     * @param node Target node for navigation
     * @param avoidCoordinates List of coordinates which should be avoided in path finding, if possible
     */
    public void navigateTo(IControllerNode node, List<RealVector> avoidCoordinates) {
        // Check for external inputs
        if (externalInputs.containsKey(BusEntry.CONSTANT_WHEELBASE) && externalInputs.containsKey(BusEntry.SENSOR_GPS_COORDINATES)) {
            Log.warning("Navigation Block: navigateTo no valid gpsCoordinates or wheelbase");
            return;
        }

        // Set last navigation target
        this.lastNavigationTarget = Optional.of(node);

        // Process navigation target without avoiding coordinates for reference
        Map<String, Object> navigationInputs = new LinkedHashMap<>();
        navigationInputs.put(NavigationEntry.MAP_ADJACENCY_LIST.toString(), WorldModel.getInstance().getControllerMap().getAdjacencies());
        navigationInputs.put(NavigationEntry.CONSTANT_WHEELBASE.toString(), externalInputs.get(BusEntry.CONSTANT_WHEELBASE));
        navigationInputs.put(NavigationEntry.GPS_COORDINATES.toString(), externalInputs.get(BusEntry.SENSOR_GPS_COORDINATES));
        navigationInputs.put(NavigationEntry.TARGET_NODE.toString(), node);
        this.functionBlock.setInputs(navigationInputs);

        Duration timeDelta = Duration.between(lastUpdate, this.getSimulator().getSimulationTime());
        double timeDeltaMs = timeDelta.toMillis();
        timeDeltaMs += (timeDelta.getNano()/1000_000d) % 1;
        functionBlock.execute(timeDeltaMs);

        // Stop processing if trajectory or avoiding coordinate list is empty
        if (functionBlock.getOutputs().get(NavigationEntry.DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()) == null) {
            return;
        }

        List<Vertex> trajectoryWithoutAvoiding = (List<Vertex>)(functionBlock.getOutputs().get(NavigationEntry.DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()));

        // Compare distance to final destination to compare quality of trajectories
        RealVector endTarget = new ArrayRealVector(new double[]{node.getPoint().getX(), node.getPoint().getY(), node.getPoint().getZ()});
        double endTargetDistanceWithoutAvoiding = trajectoryWithoutAvoiding.get(trajectoryWithoutAvoiding.size() - 1).getPosition().getDistance(endTarget);

        // Compute trajectory with avoiding coordinates on a copied adjacency list
        ArrayList<IAdjacency> adjacencyFiltered = new ArrayList<>(WorldModel.getInstance().getControllerMap().getAdjacencies());
        ArrayList<IAdjacency> adjacencyRemove = new ArrayList<>();
        Set<Long> filterOsmIds = new HashSet<>();

        // Find OSM IDs with minimal distance to coordinates to be avoided
        for (RealVector pos : avoidCoordinates) {
            double minDistSq = Double.MAX_VALUE;
            long minOsmId = -1L;

            for (IAdjacency adjacency : adjacencyFiltered) {
                RealVector posAdjacency1 = new ArrayRealVector(new double[]{adjacency.getNode1().getPoint().getX(), adjacency.getNode1().getPoint().getY(), adjacency.getNode1().getPoint().getZ()});
                RealVector posAdjacency2 = new ArrayRealVector(new double[]{adjacency.getNode2().getPoint().getX(), adjacency.getNode2().getPoint().getY(), adjacency.getNode2().getPoint().getZ()});

                // Compute square distances manually here, cheaper than distance since no sqrt is needed for minimum
                double distSq1 = (pos.getEntry(0) - posAdjacency1.getEntry(0)) * (pos.getEntry(0) - posAdjacency1.getEntry(0)) + (pos.getEntry(1) - posAdjacency1.getEntry(1)) * (pos.getEntry(1) - posAdjacency1.getEntry(1)) + (pos.getEntry(2) - posAdjacency1.getEntry(2)) * (pos.getEntry(2) - posAdjacency1.getEntry(2));
                double distSq2 = (pos.getEntry(0) - posAdjacency2.getEntry(0)) * (pos.getEntry(0) - posAdjacency2.getEntry(0)) + (pos.getEntry(1) - posAdjacency2.getEntry(1)) * (pos.getEntry(1) - posAdjacency2.getEntry(1)) + (pos.getEntry(2) - posAdjacency2.getEntry(2)) * (pos.getEntry(2) - posAdjacency2.getEntry(2));

                if (distSq1 < minDistSq) {
                    minDistSq = distSq1;
                    minOsmId = adjacency.getNode1().getOsmId();
                }

                if (distSq2 < minDistSq) {
                    minDistSq = distSq2;
                    minOsmId = adjacency.getNode2().getOsmId();
                }
            }

            if (minOsmId > 0) {
                filterOsmIds.add(minOsmId);
            }
        }

        // Find adjacency entries with OSM Ids to be removed
        for (IAdjacency adjacency : adjacencyFiltered) {
            if (filterOsmIds.contains(adjacency.getNode1().getOsmId()) || filterOsmIds.contains(adjacency.getNode2().getOsmId())) {
                adjacencyRemove.add(adjacency);
            }
        }

        // Remove all adjacency entries to be filtered out
        adjacencyFiltered.removeAll(adjacencyRemove);

        // Process navigation target without avoiding coordinates for reference
        //overwrite adjacency list
        navigationInputs.put(NavigationEntry.MAP_ADJACENCY_LIST.toString(), adjacencyFiltered);
        this.functionBlock.setInputs(navigationInputs);
        functionBlock.execute(timeDeltaMs);

        // If trajectory with avoiding is null or empty, just set original result without avoiding
        List<Vertex> trajectoryWithAvoiding = (List<Vertex>)(functionBlock.getOutputs().get(NavigationEntry.DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()));
        if (trajectoryWithAvoiding == null || trajectoryWithAvoiding.isEmpty()) {
            trajectory = trajectoryWithAvoiding;
        }
        else{
            // Compare distance to final destination to compare quality of trajectories
            double endTargetDistanceWithAvoiding = trajectoryWithAvoiding.get(trajectoryWithAvoiding.size() - 1).getPosition().getDistance(endTarget);

            // Check if end target distance with avoiding is roughly as good as without avoiding
            // If yes then set new trajectory with avoiding, otherwise use old one without avoiding
            if (endTargetDistanceWithAvoiding - 5.0 <= endTargetDistanceWithoutAvoiding) {
                trajectory = trajectoryWithAvoiding;
            }
            else{
                trajectory = trajectoryWithoutAvoiding;
            }
        }
        afterTrajectoryUpdate();
        //8 long 8 double x*8 double vector
        int vertexLength = 8 + 8 + trajectory.get(0).getPosition().getDimension() * 8;
        this.sendMessage(trajectory,trajectory.size()*vertexLength, BusEntry.NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, this.getSimulator().getSimulationTime());
    }


    /**
     * Internal function that is called after an trajectory update was performed
     */
    private void afterTrajectoryUpdate() {
        // Get current trajectory
        List<Vertex> trajectory = getTrajectory();
        if (trajectory.isEmpty()) {
            return;
        }

        // Add intersection node information to each vertex in the trajectory
        Set<OsmNode> intersectionNodes = IntersectionFinder.getInstance().getIntersections();
        for (Vertex vertex : trajectory) {
            for (OsmNode intersectionNode : intersectionNodes) {
                if (vertex.getOsmId() == intersectionNode.getId()) {
                    vertex.setIntersectionNode(true);
                }
            }
        }
    }


    /**
     * Get current trajectory of the vehicle, if available. Otherwise return empty list.
     *
     * @return Current trajectory of the vehicle, if not available return empty list
     */
    public List<Vertex> getTrajectory() {
        // Check if trajectory is available and return copy if valid
        if (trajectory != null) {
            ArrayList<Vertex> originalList = (ArrayList<Vertex>)(trajectory);
            return new ArrayList<>(originalList);
        }

        // Fallback to empty list
        return new ArrayList<>();
    }

    public NavigationBlock getNavigationBlock(){
        return functionBlock;
    }

    public Optional<IControllerNode> getLastNavigationTarget(){
        return lastNavigationTarget;
    }
}
