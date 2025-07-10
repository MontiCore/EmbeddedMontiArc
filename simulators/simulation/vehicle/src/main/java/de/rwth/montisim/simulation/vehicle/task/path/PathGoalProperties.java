/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task.path;

import java.util.List;
import java.util.Optional;
import java.util.Vector;
import java.util.stream.Collectors;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap.Node;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.task.GoalProperties;

@Typed("path")
public class PathGoalProperties extends GoalProperties {
    // One of the following three should be specified
    // These can contain only one point which will be the target point
    public Vector<Long> path_osm; // path as a list of osm IDs
    public Vector<Coordinates> path_lonlat; // global coordinate
    public Vector<Vec2> path; // local coordinate

    // if a vehicle arrives location within this range of a point,
    // the point will be considered reached by the vehicle.
    public double range = 2.0; // In meters


    // public PathGoal(List<Long> pathOsmIds, LTLOperator ltlOperator, double range) {
    //     this.ltlOperator = ltlOperator;
    //     this.pathOsmIds = pathOsmIds;
    //     this.range = range;
    //     this.currDestIdx = 0;
    // }

    // public PathGoal(LTLOperator ltlOperator, double range, List<Coordinates> coordinates) {
    //     this.ltlOperator = ltlOperator;
    //     this.pathLonLat = coordinates;
    //     this.range = range;
    //     this.currDestIdx = 0;
    // }

    // public PathGoal(LTLOperator ltlOperator, List<Vec2> path, double range) {
    //     super();
    //     this.ltlOperator = ltlOperator;
    //     this.path = path;
    //     this.range = range;
    //     this.currDestIdx = 0;
    // }


    public PathGoalProperties reach(Vec2 coord) {
        if (path == null) {
            path = new Vector<>();
        }
        path.add(coord);
        return this;
    }

    public PathGoalProperties reach(double x, double y) {
        if (path == null) {
            path = new Vector<>();
        }
        path.add(new Vec2(x, y));
        return this;
    }

    public PathGoalProperties reach(Coordinates lonlat) {
        if (path_lonlat == null) {
            path_lonlat = new Vector<>();
        }
        path_lonlat.add(lonlat);
        return this;
    }

    public PathGoalProperties reach(long osmId) {
        if (path_osm == null) {
            path_osm = new Vector<>();
        }
        path_osm.add(osmId);
        return this;
    }

    public PathGoalProperties withinRange(double rangeMeter) {
        this.range = rangeMeter;
        return this;
    }

    @Override
    public PathGoal build(Vehicle vehicle, Optional<OsmMap> map, Optional<World> world) {
        int spec = 0;
        if (path != null && path.size() > 0) ++spec;
        if (path_osm != null && path_osm.size() > 0) ++spec;
        if (path_lonlat != null && path_lonlat.size() > 0) ++spec;
        if (spec == 0)
            throw new IllegalArgumentException("PathGoalProperties has no path specified (path, path_osm or path_lonlat entry).");
        if (spec > 1)
            throw new IllegalArgumentException("PathGoalProperties multiple paths specified (path, path_osm or path_lonlat entry).");

        Vector<Vec2> finalPath;
        // Convert paths as needed
        if (path_lonlat != null && path_lonlat.size() > 0) {
            if (!world.isPresent()) throw new IllegalArgumentException("World missing from BuildContext for PathGoal");
            finalPath = convertLonLatPath(path_lonlat, world.get());
        } else if (path_osm != null && path_osm.size() > 0) {
            if (!map.isPresent()) throw new IllegalArgumentException("Osm Map missing from BuildContext for PathGoal");
            if (!world.isPresent()) throw new IllegalArgumentException("World missing from BuildContext for PathGoal");
            finalPath = convertLonLatPath(convertOsmPath(path_osm, map.get()), world.get());
        } else {
            finalPath = path;
        }

        return new PathGoal(this, finalPath);
    }

    private static List<Coordinates> convertOsmPath(Vector<Long> path_osm, OsmMap map) {
        if (map == null) throw new IllegalArgumentException("Trying to resolve osm-based path without OsmMap.");
        return path_osm.stream().map(
                osmId -> {
                    Node node = map.getNode(osmId);
                    if (node == null) throw new IllegalArgumentException("Could not find OSM-node: " + osmId);
                    return node.coords;
                }
        ).collect(Collectors.toList());
    }

    private Vector<Vec2> convertLonLatPath(List<Coordinates> pathLonLat, World world) {
        if (world == null) throw new IllegalArgumentException("Trying to convert lon/lat path without World.");
        List<Vec2> path = pathLonLat.stream().map(coord -> {
            Vec2 point = new Vec2();
            world.converter.get().coordsToMeters(coord, point);
            return point;
        }).collect(Collectors.toList());
        return new Vector<>(path);
    }
}
