/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import java.util.ArrayList;
import java.util.List;

/**
 * This Class represents streets
 */
public class Street extends EnvObject {
    public enum StreetTypes {MOTORWAY, A_ROAD, STREET, LIVING_STREET}

    ;
    public static final double STREET_WIDTH = 6;

    public enum StreetPavements {PAVED, UNPAVED, QUALITY, STONE, DIRT, GRASS}

    ;

    private boolean isOneWay;
    private Number speedLimit;
    private List<EnvNode> intersections;
    private StreetTypes streetType;
    private StreetPavements streetPavements;


    public Street(List<EnvNode> nodes, Number speedLimit, List<EnvNode> intersections, boolean isOneWay) {
        super(nodes, EnvTag.STREET);
        init(speedLimit, intersections, isOneWay);
    }

    public Street(List<EnvNode> nodes, Number speedLimit, List<EnvNode> intersections, long osmId, boolean isOneWay) {
        super(nodes, EnvTag.STREET, osmId);
        init(speedLimit, intersections, isOneWay);
    }


    public Street(List<EnvNode> nodes, Number speedLimit, List<EnvNode> intersections, long osmId, boolean isOneWay, StreetTypes streetType) {
        super(nodes, EnvTag.STREET, osmId);
        init(speedLimit, intersections, isOneWay, streetType);
    }


    public Street(List<EnvNode> nodes, Number speedLimit, List<EnvNode> intersections, long osmId, boolean isOneWay, StreetTypes streetType, StreetPavements streetPavements) {
        super(nodes, EnvTag.STREET, osmId);
        init(speedLimit, intersections, isOneWay, streetType, streetPavements);
    }

    private void init(Number speedLimit, List<EnvNode> intersections, boolean isOneWay) {
        this.speedLimit = speedLimit;
        this.isOneWay = isOneWay;
        if (intersections == null) {
            this.intersections = new ArrayList<>();
        } else {
            this.intersections = new ArrayList<>(intersections);
        }
    }


    private void init(Number speedLimit, List<EnvNode> intersections, boolean isOneWay, StreetTypes streetType) {
        this.speedLimit = speedLimit;
        this.isOneWay = isOneWay;
        this.streetType = streetType;
        if (intersections == null) {
            this.intersections = new ArrayList<>();
        } else {
            this.intersections = new ArrayList<>(intersections);
        }
    }

    private void init(Number speedLimit, List<EnvNode> intersections, boolean isOneWay, StreetTypes streetType, StreetPavements streetPavements) {
        this.speedLimit = speedLimit;
        this.isOneWay = isOneWay;
        this.streetType = streetType;
        this.streetPavements = streetPavements;
        if (intersections == null) {
            this.intersections = new ArrayList<>();
        } else {
            this.intersections = new ArrayList<>(intersections);
        }
    }

    public String toString() {
        return this.nodes.toString();
    }

    public Number getSpeedLimit() {
        return speedLimit;
    }

    public List<EnvNode> getIntersections() {
        return intersections;
    }

    public Number getStreetWidth() {
        return STREET_WIDTH;
    }

    public boolean isOneWay() {
        return isOneWay;
    }

    public StreetTypes getStreetType() {
        return streetType;
    }

    public StreetPavements getStreetPavement() {
        return streetPavements;
    }

}
