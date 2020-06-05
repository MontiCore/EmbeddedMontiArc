/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.map;

public class Adjacency {

    public ControllerNode node1;
    public ControllerNode node2;

    // public long streetId;

    public double distance;

    public Adjacency(ControllerNode node1, ControllerNode node2) {
        this.node1 = node1;
        this.node2 = node2;
        this.distance = node1.pos.distance(node2.pos);
    }

}
