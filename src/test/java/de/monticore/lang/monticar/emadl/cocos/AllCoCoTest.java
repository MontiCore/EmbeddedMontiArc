/**
 *
 *  ******************************************************************************
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
package de.monticore.lang.monticar.emadl.cocos;

import de.monticore.lang.monticar.emadl._cocos.EMADLCocos;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertTrue;

public class AllCoCoTest extends AbstractCoCoTest {
    String baseDir="src/test/resources";

    @Test
    public void testCoCosSimulator() throws IOException {
        testModel("SimpleComponent");
        testModel("AlexnetFixedParameters");
        testModel("Alexnet");
        testModel("ComponentInstanceTest");
        testModel("TargetCPP");

        //testModel("","simulator.MainController");
        //testModel("","simulator.SteerController");
        //testModel("","simulator.SteeringAngleCalculator");
        //testModel("","simulator.DistanceToTrajectoryCalculator");

    }

    private void testModel(String modelName) {
        checkValid("",modelName);
    }

    private void testInvalidModel(String modelName, int numExpectedFindings, String... expectedErrorCodes) {
        ExpectedErrorInfo errorInfo = new ExpectedErrorInfo(numExpectedFindings, expectedErrorCodes);
        checkInvalid(EMADLCocos.createChecker(), getAstNode("", modelName), errorInfo);
    }

}