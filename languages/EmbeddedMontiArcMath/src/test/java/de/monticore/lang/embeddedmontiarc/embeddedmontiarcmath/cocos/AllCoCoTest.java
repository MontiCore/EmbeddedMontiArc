/* (c) https://github.com/MontiCore/monticore */
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
 */
public class AllCoCoTest extends AbstractCoCoTest {
    String baseDir="src/test/resources/emam";
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