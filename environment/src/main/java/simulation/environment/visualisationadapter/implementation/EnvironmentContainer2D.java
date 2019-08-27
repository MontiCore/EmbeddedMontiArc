/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.visualisationadapter.implementation;

import commons.simulation.IPhysicalVehicle;
import javafx.geometry.Point2D;
import javafx.geometry.Point3D;
import simulation.environment.object.House;
import simulation.environment.visualisationadapter.interfaces.*;

import java.util.ArrayList;
import java.util.Collection;

/**
 * Created by lukas on 12.01.17.
 * A container which contains all objects in the environment
 */

public class EnvironmentContainer2D implements VisualisationEnvironmentContainer {

    private Collection<EnvStreet> streets;
    private Collection<Building> buildings;
    private Collection<House> houses;

    private Collection<EnvNode> trees;
    private Collection<Waterway> waterway;
    private EnvBounds bounds = null;

    private Collection<IPhysicalVehicle> vehicles;

    private Point3D midPoint;
    private double[][] heightMap;
    private double heightMapDeltaX;
    private double heightMapDeltaY;
    private Point2D heightMapMinPoint;
    private Point2D heightMapMaxPoint;

    public EnvironmentContainer2D(EnvBounds bounds, Collection<EnvStreet> streets, Collection<Building> buildings, Collection<Waterway> waterway) {
        this.bounds = bounds;
        this.streets = streets;
        this.buildings = buildings;
        this.waterway = waterway;
    }    

    public Collection<IPhysicalVehicle> getVehicles(){
        return this.vehicles;
    }


    @Override
    public Collection<Waterway> getWaterway() {
        return this.waterway;
    }

    @Override
    public Collection<EnvStreet> getStreets() {
        return this.streets;
    }

    @Override
    public Collection<Building> getBuildings() { return this.buildings; }

    public Collection<House> getHouses() { return this.houses; }

    @Override
    public Collection<EnvNode> getTrees() { return this.trees; }

    public void setTrees(Collection<EnvNode> trees) {
        this.trees = trees;
    }

    @Override
    public EnvBounds getBounds() { return this.bounds; }

    @Override
    public Point3D getMidpoint() {
        return this.midPoint;
    }

    @Override
    public double[][] getHeightMap() {
        return this.heightMap;
    }

    @Override
    public Point2D getHeightMapMinPoint() {
        return heightMapMinPoint;
    }

    @Override
    public Point2D getHeightMapMaxPoint() {
        return heightMapMaxPoint;
    }

    @Override
    public double getHeightMapDeltaX() {
        return heightMapDeltaX;
    }

    @Override
    public double getHeightMapDeltaY() {
        return heightMapDeltaY;
    }

    public void setHeightMapDelta(double deltaX, double deltaY) {
        heightMapDeltaX = deltaX;
        heightMapDeltaY = deltaY;
    }

    public void setHeightMapMinMax(Point2D minPoint, Point2D maxPoint) {
        heightMapMinPoint = minPoint;
        heightMapMaxPoint = maxPoint;
    }

    public void setHeightMap(double[][] heightMap) {
        this.heightMap = heightMap;
        //calculate midpoint
        Point3D maxPoint = new Point3D(this.bounds.getMaxX(), this.bounds.getMaxY(), this.bounds.getMaxZ());
        Point3D minPoint = new Point3D(this.bounds.getMinX(), this.bounds.getMinY(), this.bounds.getMinY());

        this.midPoint = maxPoint.midpoint(minPoint);
    }

    public void setBounds(EnvBounds bounds) {
        this.bounds = bounds;
    }
}
