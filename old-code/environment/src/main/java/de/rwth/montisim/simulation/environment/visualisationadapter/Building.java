/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import de.rwth.montisim.simulation.environment.visualisationadapter.EnvTag;

import java.util.List;

public class Building extends EnvObject {
    public enum BuildingTypes {UNIVERSITY, HOUSE, GARAGES, CHURCH}

    public BuildingTypes buildingTypes;

    public Building(List<EnvNode> nodes, EnvTag tag) {
        super(nodes, tag);
    }

    public Building(List<EnvNode> nodes, EnvTag tag, long osmId) {
        super(nodes, tag, osmId);
    }

    public Building(List<EnvNode> nodes, long osmId) {
        super(nodes, EnvTag.BUILDING, osmId);
    }

    public BuildingTypes getBuildingTypes() {
        return buildingTypes;
    }

    public String toString() {
        return this.nodes.toString();
    }
}
