/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam;

import de.monticore.lang.monticar.visualization.emam.application.Application;
import org.junit.Test;

import java.nio.file.Paths;

public class ApplicationTest {
    @Test
    public void testAutoPilot() {
        String modelPath = Paths.get("src/test/resources/models/AutoPilot").toAbsolutePath().toString();
        String outputPath = Paths.get("target/generated-sources/application/AutoPilot").toAbsolutePath().toString();
        String[] args = {
            "-m", "de.rwth.armin.modeling.autopilot.autopilot",
            "-mp",  modelPath,
            "-out", outputPath
        };

        Application.main(args);
    }

    @Test
    public void testElkTestController() {
        String modelPath = Paths.get("src/test/resources/models/ElkTestController").toAbsolutePath().toString();
        String outputPath = Paths.get("target/generated-sources/application/ElkTestController").toAbsolutePath().toString();
        String[] args = {
            "-m", "controller04.mainController",
            "-mp",  modelPath,
            "-out", outputPath
        };

        Application.main(args);
    }

    @Test
    public void testObjectDetector() {
        String modelPath = Paths.get("src/test/resources/models/ObjectDetector").toAbsolutePath().toString();
        String outputPath = Paths.get("target/generated-sources/application/ObjectDetector").toAbsolutePath().toString();
        String[] args = {
            "-m", "detection.objectDetector4",
            "-mp",  modelPath,
            "-out", outputPath
        };

        Application.main(args);
    }

    @Test
    public void testPacMan() {
        String modelPath = Paths.get("src/test/resources/models/PacMan").toAbsolutePath().toString();
        String outputPath = Paths.get("target/generated-sources/application/PacMan").toAbsolutePath().toString();
        String[] args = {
            "-m", "de.rwth.pacman.pacManWrapper",
            "-mp",  modelPath,
            "-out", outputPath
        };

        Application.main(args);
    }

    @Test
    public void testParkingController() {
        String modelPath = Paths.get("src/test/resources/models/ParkingController").toAbsolutePath().toString();
        String outputPath = Paths.get("target/generated-sources/application/ParkingController").toAbsolutePath().toString();
        String[] args = {
            "-m", "controller03.mainController",
            "-mp",  modelPath,
            "-out", outputPath
        };

        Application.main(args);
    }

    @Test
    public void testSuperMario() {
        String modelPath = Paths.get("src/test/resources/models/SuperMario").toAbsolutePath().toString();
        String outputPath = Paths.get("target/generated-sources/application/SuperMario").toAbsolutePath().toString();
        String[] args = {
            "-m", "de.rwth.supermario.superMarioWrapper",
            "-mp",  modelPath,
            "-out", outputPath
        };

        Application.main(args);
    }
}
