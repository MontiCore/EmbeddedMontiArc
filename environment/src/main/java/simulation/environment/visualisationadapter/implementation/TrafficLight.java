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

import javafx.geometry.Point3D;
import simulation.environment.visualisationadapter.interfaces.SignTypeAndState;
import simulation.environment.visualisationadapter.interfaces.StreetSign;

/**
 * Created by lukas on 10.03.17.
 */
public class TrafficLight implements StreetSign{

    public static long classId = 0;

    private SignTypeAndState type;
    private SignTypeAndState state;

    private long id;

    private Point3D p1;
    private Point3D p2;

    boolean isOne;
    boolean isTwo;



    public TrafficLight() {
        this.type = SignTypeAndState.TRAFFIC_LIGHT;
        this.state = SignTypeAndState.TRAFFIC_LIGHT_RED;
        this.id = classId++;
        this.isOne = this.isTwo = false;
    }

    public TrafficLight(long id) {
        this.type = SignTypeAndState.TRAFFIC_LIGHT;
        this.state = SignTypeAndState.TRAFFIC_LIGHT_RED;
        this.id = id;
        this.isOne = this.isTwo = false;

    }

    @Override
    public SignTypeAndState getSignState() {
        return this.state;
    }

    @Override
    public SignTypeAndState getType() {
        return this.type;
    }

    @Override
    public long getId() {
        return this.id;
    }

    @Override
    public boolean isOne() {
        return this.isOne;
    }

    @Override
    public boolean isTwo() {
        return this.isTwo;
    }

    @Override
    public double getX1() {
        if(this.isOne) {
            return p1.getX();
        } else {
            return Double.MIN_VALUE;
        }
    }

    @Override
    public double getY1() {
        if(this.isOne) {
            return p1.getY();
        } else {
            return Double.MIN_VALUE;
        }
    }

    @Override
    public double getZ1() {
        if(this.isOne) {
            return p1.getZ();
        } else {
            return Double.MIN_VALUE;
        }    }

    @Override
    public double getX2() {
        if(this.isTwo) {
            return p2.getX();
        } else {
            return Double.MIN_VALUE;
        }
    }

    @Override
    public double getY2() {
        if(this.isTwo) {
            return p2.getY();
        } else {
            return Double.MIN_VALUE;
        }    }

    @Override
    public double getZ2() {
        if(this.isTwo) {
            return p2.getZ();
        } else {
            return Double.MIN_VALUE;
        }    }

    @Override
    public void setOne(Point3D p1) {
        this.p1 = p1;
        this.isOne = (p1 != null);
    }

    @Override
    public void setTwo(Point3D p2) {
        this.p2 = p2;
        this.isTwo = (p2 != null);
    }


    public void setState(SignTypeAndState newState) {
        this.state = newState;
    }

}