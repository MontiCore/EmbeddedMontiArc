/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.visualisationadapter.implementation;

import simulation.environment.visualisationadapter.interfaces.EnvIntersection;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvTag;
import simulation.environment.visualisationadapter.interfaces.Waterway;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class Waterway2D extends EnvObject2D implements Waterway {



    public Waterway.WaterTypes waterType;

    public Waterway2D(List<EnvNode> nodes) {
        super(nodes, EnvTag.WATERWAY);
    }

    public Waterway2D(List<EnvNode> nodes,  long osmId) {
        super(nodes, EnvTag.WATERWAY, osmId);

    }

    public Waterway2D(List<EnvNode> nodes,  long osmId, Waterway.WaterTypes waterType) {
        super(nodes, EnvTag.WATERWAY, osmId);

    }





    public String toString() {
        return this.nodes.toString();
    }





    @Override
    public Number getWaterwayWidth() {
        return Waterway.RIVER_WIDTH;
    }



    @Override
    public Waterway.WaterTypes getWaterType(){ return waterType; }
}
