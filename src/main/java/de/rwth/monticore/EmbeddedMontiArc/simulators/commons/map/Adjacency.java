/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map;

/**
 * Created by lukas on 31.01.17.
 */
public class Adjacency implements IAdjacency{

    private IControllerNode node1;
    private IControllerNode node2;

    private long streetId;

    private double distance;


    public Adjacency(IControllerNode node1, IControllerNode node2) {
        this.node1 = node1;
        this.node2 = node2;
        this.distance = node1.getPoint().distance(node2.getPoint());
    }

    @Override
    public IControllerNode getNode1() {
        return this.node1;
    }

    @Override
    public IControllerNode getNode2() {
        return this.node2;
    }

    @Override
    public double getDistance() {
        return this.distance;
    }
}
