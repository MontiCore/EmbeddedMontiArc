/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.environment.visualisationadapter.implementation;

import javafx.geometry.Point2D;
import javafx.geometry.Point3D;
import simulation.environment.visualisationadapter.interfaces.*;
import java.util.Collection;

/**
 * Created by lukas on 12.01.17.
 * A container which contains all objects in the environment
 */

public class EnvironmentContainer2D implements VisualisationEnvironmentContainer {

    private Collection<EnvStreet> streets;
    private Collection<Building> buildings;
    private Collection<EnvNode> trees;
    private EnvBounds bounds = null;

    private Point3D midPoint;
    private double[][] heightMap;
    private double heightMapDeltaX;
    private double heightMapDeltaY;
    private Point2D heightMapMinPoint;
    private Point2D heightMapMaxPoint;

    public EnvironmentContainer2D(EnvBounds bounds, Collection<EnvStreet> streets, Collection<Building> buildings) {
        this.bounds = bounds;
        this.streets = streets;
        this.buildings = buildings;

    }


    @Override
    public Collection<EnvStreet> getStreets() {
        return this.streets;
    }

    @Override
    public Collection<Building> getBuildings() {
        return this.buildings;
    }

    @Override
    public Collection<EnvNode> getTrees() {
        return trees;
    }

    public void setTrees(Collection<EnvNode> trees) {
        this.trees = trees;
    }

    @Override
    public EnvBounds getBounds() {
        return this.bounds;
    }

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