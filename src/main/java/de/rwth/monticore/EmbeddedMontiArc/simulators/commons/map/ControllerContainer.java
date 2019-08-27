/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map;

import java.util.ArrayList;

/**
 * Created by lukas on 13.02.17.
 */
public class ControllerContainer {

    private ArrayList<IAdjacency> adjacencies;

    private PathListener listener;

    public ControllerContainer(ArrayList<IAdjacency> adjacencies, PathListener listener) {
        this.adjacencies = adjacencies;
        this.listener = listener;
    }

    public PathListener getListener() {
        return this.listener;
    }

    public ArrayList<IAdjacency> getAdjacencies() {
        return this.adjacencies;
    }

}
