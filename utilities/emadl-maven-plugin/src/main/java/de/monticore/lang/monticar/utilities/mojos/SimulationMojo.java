package de.monticore.lang.monticar.utilities.mojos;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import java.util.Set;
import java.util.stream.Stream;
import java.util.stream.Collectors;

import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import de.rwth.montisim.hardware_emulator.CppBridge;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.simulation.simulator.SimulationCLI;

/**
 * Runs the (autopilot) EMA model in MontiSim based on a list of scenarios.
 */
@Mojo(name = "simulate")
public class SimulationMojo extends AbstractMojo {

    static {
        TypedHardwareEmu.registerTypedHardwareEmu();
    }
    /**
     * The path to a scenarios folder.
     * All scenarios in the folder will be executed.
     */
    @Parameter(defaultValue = "scenarios")
    private String scenarioFolder;
    /**
     * The path to the maps used by the scenarios.
     */
    @Parameter(defaultValue = "maps")
    private String mapsFolder;
    /**
     * The path to the folder containing the compiled *library adapter* of the EMA model.
     */
    @Parameter(defaultValue = "autopilots")
    private String autopilotsFolder;

    @Override
    public void execute() throws MojoExecutionException, MojoFailureException {
        try {
            boolean anyFailed = false;
            CppBridge.init("{\"softwares_folder\": \""+autopilotsFolder+"\"}");
            Set<Path> scenarios = listFilesUsingFilesList(Paths.get(scenarioFolder));
            for (Path p : scenarios) {
                getLog().info("Simulating scenario: "+p.toString());
                TaskStatus res = SimulationCLI.runSimulationFromFile(p.toString(), mapsFolder);
                if (res != TaskStatus.SUCCEEDED) anyFailed = true;
            }
            if (anyFailed) throw new MojoExecutionException("Some simulations failed.");
        } catch (Exception e) {
            throw new MojoFailureException("Error: ", e);
        }
    }
    
    // From https://www.baeldung.com/java-list-directory-files
    public Set<Path> listFilesUsingFilesList(Path dir) throws IOException {
        try (Stream<Path> stream = Files.list(dir)) {
            return stream
              .filter(file -> !Files.isDirectory(file))
              .collect(Collectors.toSet());
        }
    }
}
