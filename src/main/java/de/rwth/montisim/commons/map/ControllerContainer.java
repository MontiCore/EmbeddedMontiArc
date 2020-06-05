/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.map;

import java.util.ArrayList;

/**
 * Created by lukas on 13.02.17.
 */
public class ControllerContainer {

    private ArrayList<Adjacency> adjacencies;

    private PathListener listener;

    public ControllerContainer(ArrayList<Adjacency> adjacencies, PathListener listener) {
        this.adjacencies = adjacencies;
        this.listener = listener;
    }

    public PathListener getListener() {
        return this.listener;
    }

    public ArrayList<Adjacency> getAdjacencies() {
        return this.adjacencies;
    }

}
