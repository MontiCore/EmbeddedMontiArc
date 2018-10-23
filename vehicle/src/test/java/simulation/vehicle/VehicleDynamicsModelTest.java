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

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.util.Log;

/**
 * Class that tests the VehicleDynamicsModel class
 */
public class VehicleDynamicsModelTest {
    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test(expected = IllegalStateException.class)
    public void initialiseFail(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialise();
        model.initialise();
    }

    @Test
    public void doStepNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialise();
        model.doStep(0.033);
    }

    @Test(expected = IllegalStateException.class)
    public void doStepFail(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.doStep(0.033);
    }

    @Test
    public void setParameterNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.setParameter("m", 1750);
    }

    @Test(expected = IllegalStateException.class)
    public void setParameterFailInitialised(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialise();
        model.setParameter("m", 1750);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setParameterFailWrongName(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.setParameter("wrong", 1750);
    }

    @Test
    public void setInputNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialise();
        model.setInput("delta_1", 0.3);
    }

    @Test(expected = IllegalStateException.class)
    public void setInputFailUninitialised(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.setInput("delta_1", 0.3);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setInputFailWrongName(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialise();
        model.setInput("wrong", 0.3);
    }

    @Test
    public void getValueNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialise();
        model.getValue("m");
    }

    @Test(expected = IllegalStateException.class)
    public void getValueFailUninitialised(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.getValue("m");
    }

    @Test(expected = IllegalArgumentException.class)
    public void getValueFailWrongName(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialise();
        model.getValue("wrong");
    }
}