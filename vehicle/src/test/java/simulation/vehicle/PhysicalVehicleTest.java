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
package simulation.vehicle;

import static org.junit.Assert.assertEquals;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import simulation.util.Log;
import simulation.vehicle.ModelicaPhysicalVehicle;
import simulation.vehicle.ModelicaPhysicalVehicleBuilder;
import simulation.vehicle.PhysicalVehicle;

public class PhysicalVehicleTest {

	@BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void setHeightNormal(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setHeight(2.0);
        assertEquals(2.0, physicalVehicle.getHeight(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setHeightFail(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setHeight(2.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getMassFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getMass();
    }

    @Test
    public void setMassNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
        assertEquals(1000.0, physicalVehicle.getMass(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setMass(1000.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1000.0, physicalVehicle.getMass(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFailMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelRadiusFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelRadius();
    }

    @Test
    public void setWheelRadiusNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
        assertEquals(1.0, physicalVehicle.getWheelRadius(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelRadius(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelRadius(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelRadiusMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelRadiusFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightFrontSideFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistLeftRightFrontSide();
    }

    @Test
    public void setWheelDistLeftRightFrontSideNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightFrontSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightFrontSide(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightFrontSideMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightFrontSideFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightBackSideFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistLeftRightBackSide();
    }

    @Test
    public void setWheelDistLeftRightBackSideNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightBackSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightBackSide(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightBackSideMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightBackSideFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToFrontFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistToFront();
    }

    @Test
    public void setWheelDistToFrontNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistToFront(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistToFront(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistToFront(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToFrontMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToFrontFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToBackFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistToBack();
    }

    @Test
    public void setWheelDistToBackNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistToBack(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistToBack(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistToBack(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToBackMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToBackFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
    }
}
