/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.osm;

import simulation.environment.object.ChargingStation;
import simulation.environment.visualisationadapter.implementation.*;
import simulation.environment.visualisationadapter.interfaces.*;
import simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D;
import java.util.ArrayList;

/**
 * Created by lukas on 02.02.17.
 * This class encapsulates the whole conversion process and converts all objects in a given EnvironmentContainer to kilometric units
 */
public class EnvironmentContainerConverter {
    private VisualisationEnvironmentContainer containerLongLat;

    private simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D containerMeters;

    private ApproximateConverter converter;

    private EnvBounds bounds;



    public EnvironmentContainerConverter(VisualisationEnvironmentContainer containerLongLat) {
        this.containerLongLat = containerLongLat;
        computeMinLongMinLat();
        convertLatLongToMeters();
    }

    public EnvironmentContainerConverter(VisualisationEnvironmentContainer containerLongLat, double minLong, double minLat) {
        this.containerLongLat = containerLongLat;
        converter = new ApproximateConverter(minLong, minLat);
        convertLatLongToMeters();
    }

    public ApproximateConverter getApproximateConverter() {
        return converter;
    }

    /**
     * converts all Street Nodes in the container to kilometric units
     */
    private void convertLatLongToMeters() {
        ArrayList<EnvStreet> meterStreets = new ArrayList<>();
        ArrayList<Building> meterBuildings = new ArrayList<>();
        ArrayList<Waterway> meterWaterway = new ArrayList<>();
        ArrayList<ChargingStation> meterChargingStations = new ArrayList<>();

        for(EnvStreet longLatStreet : containerLongLat.getStreets()) {
            ArrayList<EnvNode> nodes = new ArrayList<>();
            for(EnvNode node : longLatStreet.getNodes()) {
                double nLong = node.getX().doubleValue();
                double nLat = node.getY().doubleValue();


                double mY = converter.convertLatToMeters(nLat);
                double mX = converter.convertLongToMeters(nLong, nLat);

                double mZ = node.getZ().doubleValue();
                long osmId = node.getOsmId();
                nodes.add(new Node2D(mX, mY, mZ, osmId));
            }

            ArrayList<EnvIntersection> intersections = new ArrayList<>();
            for(EnvNode intersection : longLatStreet.getIntersections()) {
                double nLong = intersection.getX().doubleValue();
                double nLat = intersection.getY().doubleValue();


                double mY = converter.convertLatToMeters(nLat);
                double mX = converter.convertLongToMeters(nLong, nLat);

                double mZ = intersection.getZ().doubleValue();
                long osmId = intersection.getOsmId();
                intersections.add(new Intersection2D(mX, mY, mZ, osmId));
            }

            meterStreets.add(new Street2D(nodes, longLatStreet.getSpeedLimit(), intersections, longLatStreet.getOsmId(), longLatStreet.isOneWay(), longLatStreet.getStreetType(), longLatStreet.getStreetPavement()));
        }
        for(Building longLatBuilding : containerLongLat.getBuildings()) {
            ArrayList<EnvNode> nodes = new ArrayList<>();
            for (EnvNode node : longLatBuilding.getNodes()) {
                double nLong = node.getX().doubleValue();
                double nLat = node.getY().doubleValue();


                double mY = converter.convertLatToMeters(nLat);
                double mX = converter.convertLongToMeters(nLong, nLat);

                double mZ = node.getZ().doubleValue();
                long osmId = node.getOsmId();
                nodes.add(new Node2D(mX, mY, mZ, osmId));
            }

            meterBuildings.add(new BuildingImpl(nodes, longLatBuilding.getOsmId()));
        }

        for(Waterway longLatWaterway : containerLongLat.getWaterway()) {
            ArrayList<EnvNode> nodes = new ArrayList<>();
            for (EnvNode node : longLatWaterway.getNodes()) {
                double nLong = node.getX().doubleValue();
                double nLat = node.getY().doubleValue();


                double mY = converter.convertLatToMeters(nLat);
                double mX = converter.convertLongToMeters(nLong, nLat);

                double mZ = node.getZ().doubleValue();
                long osmId = node.getOsmId();
                nodes.add(new Node2D(mX, mY, mZ, osmId));
            }

            meterWaterway.add(new Waterway2D(nodes, longLatWaterway.getOsmId()));
        }

        for(ChargingStation chargingStation: containerLongLat.getChargingStations()) {
            EnvNode node = chargingStation.getNodes().get(0);
            double nLong = node.getX().doubleValue();
            double nLat = node.getY().doubleValue();

            double mY = converter.convertLatToMeters(nLat);
            double mX = converter.convertLongToMeters(nLong, nLat);

            double mZ = node.getZ().doubleValue();
            long osmId = node.getOsmId();

            Node2D nodeInMeter = new Node2D(mX, mY, mZ, osmId);
            meterChargingStations.add(new ChargingStation(
                    osmId, nodeInMeter, chargingStation.getCapacity(), chargingStation.getName()));
        }
        computeMinMax(meterStreets, meterBuildings, meterWaterway, meterChargingStations);
        containerMeters = new EnvironmentContainer2D(
                bounds,meterStreets, meterBuildings, meterWaterway, meterChargingStations);
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
        double minX = 0 - Waterway.RIVER_WIDTH;
        double minY = 0 - Waterway.RIVER_WIDTH;
        double minZ = 0;

        double maxX = Double.MIN_VALUE;
        double maxY = Double.MIN_VALUE;
        double maxZ = Double.MIN_VALUE;

        for(EnvStreet street: streets) {
            for(EnvNode nodes: street.getNodes()) {
                if(nodes.getX().doubleValue() > maxX) {
                    maxX = nodes.getX().doubleValue();
                }

                if(nodes.getY().doubleValue() > maxY) {
                    maxY = nodes.getY().doubleValue();
                }

                if(nodes.getZ().doubleValue() > maxZ) {
                    maxZ = nodes.getZ().doubleValue();
                }
            }
        }
        for(Building building: buildings) {
            for(EnvNode nodes: building.getNodes()) {
                if(nodes.getX().doubleValue() > maxX) {
                    maxX = nodes.getX().doubleValue();
                }

                if(nodes.getY().doubleValue() > maxY) {
                    maxY = nodes.getY().doubleValue();
                }

                if(nodes.getZ().doubleValue() > maxZ) {
                    maxZ = nodes.getZ().doubleValue();
                }
            }
        }
        for(Waterway waterway: waterways) {
            for(EnvNode nodes: waterway.getNodes()) {
                if(nodes.getX().doubleValue() > maxX) {
                    maxX = nodes.getX().doubleValue();
                }

                if(nodes.getY().doubleValue() > maxY) {
                    maxY = nodes.getY().doubleValue();
                }

                if(nodes.getZ().doubleValue() > maxZ) {
                    maxZ = nodes.getZ().doubleValue();
                }
            }
        }
        for(ChargingStation chargingStation: chargingStations) {
            for(EnvNode nodes: chargingStation.getNodes()) {
                if(nodes.getX().doubleValue() > maxX) {
                    maxX = nodes.getX().doubleValue();
                }

                if(nodes.getY().doubleValue() > maxY) {
                    maxY = nodes.getY().doubleValue();
                }

                if(nodes.getZ().doubleValue() > maxZ) {
                    maxZ = nodes.getZ().doubleValue();
                }
            }
        }

        this.bounds = new Bounds2D(minX, maxX, minY, maxY, minZ, maxZ);
    }


    /**
     * computes the minimum longitude and latitude and initialises the converter
     */
    private void computeMinLongMinLat() {
        double minLat = Double.MAX_VALUE;
        double minLong = Double.MAX_VALUE;

        for(EnvObject object : containerLongLat.getStreets()) {
            for(EnvNode node : object.getNodes()) {
                double nLat = node.getY().doubleValue();
                double nLong = node.getX().doubleValue();

                if(nLat <= minLat) {
                    minLat = nLat;
                }

                if(nLong <= minLong) {
                    minLong = nLong;
                }
            }
        }

        converter = new ApproximateConverter(minLong, minLat);
    }

    public EnvironmentContainer2D getContainer() {
        return this.containerMeters;
    }
}
