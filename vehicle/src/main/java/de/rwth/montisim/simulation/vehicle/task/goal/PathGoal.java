package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.util.List;
import java.util.stream.Collectors;


/**
 *
 */
@Typed
public class PathGoal extends Goal {
    private List<Long> pathOsmIds; // path as a list of osm IDs
    private List<Coordinates> pathLonLat; // global coordinate
    private List<Vec2> path;  // local coordinate

    // if a vehicle arrives location within this range of a point,
    // the point will be considered reached by the vehicle.
    private double range;

    private int currDestIdx; // the index of current destination (next location the vehicle should reach)

    public PathGoal() {
    }

    public PathGoal(List<Long> pathOsmIds, LTLOperator ltlOperator, double range) {
        this.ltlOperator = ltlOperator;
        this.pathOsmIds = pathOsmIds;
        this.range = range;
        this.currDestIdx = 0;
    }

    public PathGoal(LTLOperator ltlOperator, double range, List<Coordinates> coordinates) {
        this.ltlOperator = ltlOperator;
        this.pathLonLat = coordinates;
        this.range = range;
        this.currDestIdx = 0;
    }

    public PathGoal(LTLOperator ltlOperator, List<Vec2> path, double range) {
        super();
        this.ltlOperator = ltlOperator;
        this.path = path;
        this.range = range;
        this.currDestIdx = 0;
    }

    /**
     * Convert path described using OSM ID or lon/lat into a path that all waypoints are described using local coordinate system
     *
     * @param map
     * @param world
     */
    public void convertCoordinate(OsmMap map, World world) {
        if (pathOsmIds != null)
            convertOsmPath(map, world);
        else if (pathLonLat != null)
            convertLonLatPath(world);
    }

    private void convertOsmPath(OsmMap map, World world) {
        pathLonLat = pathOsmIds.stream().map(osmId ->
                // Get lon/lat of each osm node, identified by osmId
                // Throw runtime error if no matching can be found
                map.nodeTable.stream().filter(node -> node.osmID == osmId).collect(Collectors.toList()).get(0).coords
        ).collect(Collectors.toList());

        convertLonLatPath(world);
    }

    private void convertLonLatPath(World world) {
        path = pathLonLat.stream().map(coord -> {
            Vec2 point = new Vec2();
            world.converter.get().coordsToMeters(coord, point);
            return point;
        }).collect(Collectors.toList());
    }

    public void update(Vehicle v) {
        if (path == null)
            throw new RuntimeException("path from PathGoal must not be null");

        if (ltlOperator.equals(LTLOperator.EVENTUALLY)) {
            handleEventually(v);
        } else if (ltlOperator.equals(LTLOperator.NEVER)) {
            handleNever(v);
        }
    }

    private void handleEventually(Vehicle v) {
        if (currDestIdx >= path.size()) return; // skip if all destinations have been reached

        Vec2 dest = path.get(currDestIdx);
        double dist = v.physicalObject.pos.asVec2().distance(dest);

        // nothing to update if current destination have not been reached yet
        if (dist > range) {
            return;
        }

        // a coordinate in path is reached,
        // update next destination and goal status
        currDestIdx += 1;
        if (currDestIdx < path.size()) {
            updateStatus(TaskStatus.RUNNING);
        } else {
            // goal is successful if all destinations have been reached
            updateStatus(TaskStatus.SUCCEEDED);
        }
    }

    private void handleNever(Vehicle v) {
        // "never" operator expect the expression to always be false (FAILED)
        // i.e. the vehicle should never arrive in the area of any given destination
        TaskStatus status = TaskStatus.FAILED;

        for (Vec2 vec : path) {
            if (v.physicalObject.pos.distance(new Vec3(vec.x, vec.y, 0)) < range) {
                // succeeded if arrived in the area of vec
                status = TaskStatus.SUCCEEDED;
                break;
            }
        }

        updateStatus(status);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this)
            return true;
        if (obj == null || obj.getClass() != getClass())
            return false;

        PathGoal goal = (PathGoal) obj;
        if (goal.getStatus() != status)
            return false;
        if (!goal.path.equals(path))
            return false;

        return true;
    }

    public static PathGoalBuilder newBuilder() {
        return new PathGoalBuilder();
    }

    public List<Vec2> getPath() {
        return path;
    }

    public void setPath(List<Vec2> path) {
        this.path = path;
    }

    public List<Coordinates> getPathLonLat() {
        return pathLonLat;
    }

    public void setPathLonLat(List<Coordinates> pathLonLat) {
        this.pathLonLat = pathLonLat;
    }

    public List<Long> getPathOsmIds() {
        return pathOsmIds;
    }

    public void setPathOsmIds(List<Long> pathOsmIds) {
        this.pathOsmIds = pathOsmIds;
    }

    public double getRange() {
        return range;
    }

    public int getCurrDestIdx() {
        return currDestIdx;
    }
}

