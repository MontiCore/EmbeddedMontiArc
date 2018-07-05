package de.monticore.lang.monticar.visualization.emam;

import de.monticore.lang.monticar.visualization.emam.application.Application;
import org.junit.Test;

import java.nio.file.Paths;

public class ApplicationTest {
    @Test
    public void testStart() {
        String modelPath = Paths.get("src/test/resources/models").toAbsolutePath().toString();
        String outputPath = Paths.get("target/generated-sources/application").toAbsolutePath().toString();
        String[] args = { "-m", "alpha.beta.Gamma", "-mp",  modelPath, "-out", outputPath};

        Application.main(args);
    }
}
