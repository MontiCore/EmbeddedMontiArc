/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvTag;

import java.util.List;

public class Waterway extends EnvObject {

    public final double RIVER_WIDTH = 9;

    public enum WaterTypes {RIVER, STREAM, DITCH}

    ;

    public WaterTypes waterType;

    public Waterway(List<EnvNode> nodes) {
        super(nodes, EnvTag.WATERWAY);
    }

    public Waterway(List<EnvNode> nodes, long osmId) {
        super(nodes, EnvTag.WATERWAY, osmId);

    }

    public Waterway(List<EnvNode> nodes, long osmId, WaterTypes waterType) {
        super(nodes, EnvTag.WATERWAY, osmId);

    }

    public String toString() {
        return this.nodes.toString();
    }

    public Number getWaterwayWidth() {
        return RIVER_WIDTH;
    }

    public WaterTypes getWaterType() {
        return waterType;
    }
}
