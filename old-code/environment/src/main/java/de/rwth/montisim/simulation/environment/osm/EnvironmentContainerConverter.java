/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.*;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.*;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by lukas on 02.02.17.
 * This class encapsulates the whole conversion process and converts all objects in a given EnvironmentContainer to kilometric units
 */
public class EnvironmentContainerConverter {
    private VisualisationEnvironmentContainer containerLongLat;

    private simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D containerMeters;

    private ApproximateConverter converter;

    private Bounds2D bounds;


    public EnvironmentContainerConverter(VisualisationEnvironmentContainer containerLongLat) {
        this.containerLongLat = containerLongLat;
        computeMinLongMinLat();
        convertLatLongToMeters();
    }

    public EnvironmentContainerConverter(VisualisationEnvironmentContainer containerLongLat, Coordinates minCorner) {
        this.containerLongLat = containerLongLat;
        converter = new ApproximateConverter(minCorner);
        convertLatLongToMeters();
    }

    public ApproximateConverter getApproximateConverter() {
        return converter;
    }

    List<EnvNode> convertNodes(List<EnvNode> geoCoordNodes) {
        ArrayList<EnvNode> nodes = new ArrayList<>();
        for (EnvNode node : geoCoordNodes) {
            nodes.add(new Node2D(
                    new Vec3(converter.coordinatesToMeters(new Coordinates(node.point.x, node.point.y)), node.point.z),
                    node.osmId)
            );
        }
        return nodes;
    }

    /**
     * converts all Street Nodes in the container to kilometric units
     */
    private void convertLatLongToMeters() {
        ArrayList<EnvStreet> meterStreets = new ArrayList<>();
        ArrayList<Building> meterBuildings = new ArrayList<>();
        ArrayList<Waterway> meterWaterway = new ArrayList<>();
        ArrayList<ChargingStation> meterChargingStations = new ArrayList<>();

        for (EnvStreet longLatStreet : containerLongLat.getStreets()) {
            List<EnvNode> nodes = convertNodes(longLatStreet.getNodes());
            List<EnvNode> intersections = convertNodes(longLatStreet.getIntersections());
            meterStreets.add(new Street2D(nodes, longLatStreet.getSpeedLimit(), intersections, longLatStreet.getOsmId(), longLatStreet.isOneWay(), longLatStreet.getStreetType(), longLatStreet.getStreetPavement()));
        }

        for (Building longLatBuilding : containerLongLat.getBuildings()) {
            List<EnvNode> nodes = convertNodes(longLatBuilding.getNodes());
            meterBuildings.add(new BuildingImpl(nodes, longLatBuilding.getOsmId()));
        }

        for (Waterway longLatWaterway : containerLongLat.getWaterway()) {
            List<EnvNode> nodes = convertNodes(longLatWaterway.getNodes());
            meterWaterway.add(new Waterway2D(nodes, longLatWaterway.getOsmId()));
        }

        for (ChargingStation chargingStation : containerLongLat.getChargingStations()) {
            List<EnvNode> nodes = convertNodes(chargingStation.getNodes());
            EnvNode csNode = nodes.get(0);
            meterChargingStations.add(new ChargingStation(
                    csNode.osmId, csNode, chargingStation.getCapacity(), chargingStation.getName()));
        }
        computeMinMax(meterStreets, meterBuildings, meterWaterway, meterChargingStations);
        containerMeters = new EnvironmentContainer2D(
                bounds, meterStreets, meterBuildings, meterWaterway, meterChargingStations);
    }

    /**
     * computes the min and max values for x,y,z and thus the bounds of the environment
     * @param streets
     */
    private void computeMinMax(
            ArrayList<EnvStreet> streets,
            ArrayList<Building> buildings,
            ArrayList<Waterway> waterways,
            ArrayList<ChargingStation> chargingStations) {
        //assure that every street (including pavements lies in the bounds of the environment
        // double minX = 0 - Waterway.RIVER_WIDTH;
        // double minY = 0 - Waterway.RIVER_WIDTH;
        // double minZ = 0;

        Vec3 min = new Vec3(Double.MAX_VALUE);
        Vec3 max = new Vec3(Double.MIN_VALUE);

        for (EnvStreet street : streets) {
            for (EnvNode nodes : street.getNodes()) {
                Geometry.minimize(min, nodes.point);
                Geometry.maximize(max, nodes.point);
            }
        }
        for (Building building : buildings) {
            for (EnvNode nodes : building.getNodes()) {
                Geometry.minimize(min, nodes.point);
                Geometry.maximize(max, nodes.point);
            }
        }
        for (Waterway waterway : waterways) {
            for (EnvNode nodes : waterway.getNodes()) {
                Geometry.minimize(min, nodes.point);
                Geometry.maximize(max, nodes.point);
            }
        }
        for (ChargingStation chargingStation : chargingStations) {
            for (EnvNode nodes : chargingStation.getNodes()) {
                Geometry.minimize(min, nodes.point);
                Geometry.maximize(max, nodes.point);
            }
        }

        this.bounds = new Bounds2D(min, max);
    }


    /**
     * computes the minimum longitude and latitude and initialises the converter
     */
    private void computeMinLongMinLat() {
        Vec3 min = new Vec2(Double.MAX_VALUE);

        for (EnvObject object : containerLongLat.getStreets()) {
            for (EnvNode node : object.getNodes()) {
                Geometry.minimize(min, node.point);
            }
        }

        converter = new ApproximateConverter(new Coordinates(min.x, min.y));
    }

    public EnvironmentContainer2D getContainer() {
        return this.containerMeters;
    }
}
