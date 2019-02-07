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

import commons.controller.interfaces.Bus;
import commons.controller.interfaces.FunctionBlockInterface;
import commons.map.IControllerNode;
import commons.simulation.PhysicalObjectType;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.jfree.data.xy.Vector;
import org.junit.*;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.util.Log;
import simulation.util.MathHelper;
import java.util.Optional;


public class FrictionTest {
    @Test
    public static void test(){

        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) builder.buildPhysicalVehicle();
        //RealVector vector = (30,10);//(6453.37385514145,2774.6997263998,0.0);
        Assert.assertEquals(1, PhysicsEngine.calcFrictionCoefficient(physicalVehicle.getPosition()), 0);//((WorldModel) WorldModel.getInstance()).getMinimumStreetForRealVector().getObject(), false), 0);
    }
}
