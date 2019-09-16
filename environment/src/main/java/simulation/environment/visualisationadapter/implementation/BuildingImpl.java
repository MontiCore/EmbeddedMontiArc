/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.visualisationadapter.implementation;

import simulation.environment.visualisationadapter.interfaces.Building;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvTag;

import java.util.List;

public class BuildingImpl extends  EnvObject2D implements Building {

    public BuildingTypes buildingTypes;
    public BuildingImpl(List<EnvNode> nodes, EnvTag tag) {
        super(nodes, tag);
    }

    public BuildingImpl(List<EnvNode> nodes, EnvTag tag, long osmId) {
        super(nodes, tag, osmId);
    }

    public BuildingImpl(List<EnvNode> nodes, long osmId) {
        super(nodes, EnvTag.BUILDING, osmId);
    }

    @Override
    public BuildingTypes getBuildingTypes() {
        return buildingTypes;
    }

    public String toString(){
        return this.nodes.toString();
    }
}
