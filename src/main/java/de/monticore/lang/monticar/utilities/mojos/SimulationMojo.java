package de.monticore.lang.monticar.utilities.mojos;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Set;
import java.util.stream.Stream;
import java.util.stream.Collectors;

import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import de.rwth.montisim.simulation.simulator.SimulationCLI;

@Mojo(name = "simulate")
public class SimulationMojo extends AbstractMojo {
    
    @Parameter(defaultValue = "scenarios")
    private String scenarioFolder;

    @Override
    public void execute() throws MojoExecutionException, MojoFailureException {
        try {
            Set<Path> scenarios = listFilesUsingFilesList(Path.of(scenarioFolder));
            for (Path p : scenarios) {
                SimulationCLI.runSimulationFromFile(p.toString());
            }
        } catch (IOException e) {
            throw new MojoExecutionException("Error getting the scenario files", e);
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
