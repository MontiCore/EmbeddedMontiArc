/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.environment.object.House;

import java.util.Collection;

/**
 * A container which contains all objects in the environment
 */

public class EnvironmentContainer {

    private Collection<Street> streets;
    private Collection<Building> buildings;
    private Collection<House> houses;

    private Collection<EnvNode> trees;
    private Collection<Waterway> waterway;
    private Collection<ChargingStation> chargingStations;
    private EnvBounds bounds = null;

    //private Collection<IPhysicalVehicle> vehicles;

    private Vec3 midPoint;
    private double[][] heightMap;
    private double heightMapDeltaX;
    private double heightMapDeltaY;
    private Vec2 heightMapMinPoint;
    private Vec2 heightMapMaxPoint;

    public EnvironmentContainer(
            EnvBounds bounds,
            Collection<Street> streets,
            Collection<Building> buildings,
            Collection<Waterway> waterway,
            Collection<ChargingStation> chargingStations) {
        this.bounds = bounds;
        this.streets = streets;
        this.buildings = buildings;
        this.waterway = waterway;
        this.chargingStations = chargingStations;
    }

    // public Collection<IPhysicalVehicle> getVehicles(){
    //     return this.vehicles;
    // }


    public Collection<Waterway> getWaterway() {
        return this.waterway;
    }

    public Collection<ChargingStation> getChargingStations() {
        return chargingStations;
    }

    public void setChargingStations(Collection<ChargingStation> chargingStations) {
        this.chargingStations = chargingStations;
    }

    public Collection<Street> getStreets() {
        return this.streets;
    }

    public Collection<Building> getBuildings() {
        return this.buildings;
    }

    public Collection<House> getHouses() {
        return this.houses;
    }

    public Collection<EnvNode> getTrees() {
        return this.trees;
    }

    public void setTrees(Collection<EnvNode> trees) {
        this.trees = trees;
    }

    public EnvBounds getBounds() {
        return this.bounds;
    }

    public Vec3 getMidpoint() {
        return this.midPoint;
    }

    public double[][] getHeightMap() {
        return this.heightMap;
    }

    public Vec2 getHeightMapMinPoint() {
        return heightMapMinPoint;
    }

    public Vec2 getHeightMapMaxPoint() {
        return heightMapMaxPoint;
    }

    public double getHeightMapDeltaX() {
        return heightMapDeltaX;
    }

    public double getHeightMapDeltaY() {
        return heightMapDeltaY;
    }

    public void setHeightMapDelta(double deltaX, double deltaY) {
        heightMapDeltaX = deltaX;
        heightMapDeltaY = deltaY;
    }

    public void setHeightMapMinMax(Vec2 minPoint, Vec2 maxPoint) {
        heightMapMinPoint = minPoint;
        heightMapMaxPoint = maxPoint;
    }

    public void setHeightMap(double[][] heightMap) {
        this.heightMap = heightMap;
        this.midPoint = this.bounds.max.midpoint(this.bounds.min);
    }

    public void setBounds(EnvBounds bounds) {
        this.bounds = bounds;
    }
}
