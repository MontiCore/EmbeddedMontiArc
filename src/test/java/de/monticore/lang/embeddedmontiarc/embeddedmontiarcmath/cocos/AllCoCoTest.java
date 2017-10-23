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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.ParserTest;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertTrue;

/**
 * @author Sascha Schneiders
 */
public class AllCoCoTest extends AbstractCoCoTest {
    String baseDir="src/test/resources";
    @Test
    public void testCoCosSimulator() throws IOException {
        testModel("","simulator.MainController");
        testModel("","simulator.SteerController");
        testModel("","simulator.SteeringAngleCalculator");
        //testModel("","simulator.DistanceToTrajectoryCalculator");

    }

    private void testModel(String modelPath, String modelName) {
        checkValid(modelPath,modelName);
    }

}
