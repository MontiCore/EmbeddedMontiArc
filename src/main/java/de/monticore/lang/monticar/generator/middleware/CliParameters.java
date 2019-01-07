package de.monticore.lang.monticar.generator.middleware;

import java.util.Set;

public class CliParameters {
    private String modelsDir;
    private String outputDir;
    private String rootModel;
    private Set<String> generators;
    private String emadlBackend;

    public CliParameters() {
    }

    public CliParameters(String modelsDir, String outputDir, String rootModel, Set<String> generators) {
        this(modelsDir, outputDir, rootModel, generators, "MXNET");
    }

    public CliParameters(String modelsDir, String outputDir, String rootModel, Set<String> generators, String emadlBackend) {
        this.modelsDir = modelsDir;
        this.outputDir = outputDir;
        this.rootModel = rootModel;
        this.generators = generators;
        this.emadlBackend = emadlBackend;
    }

    public String getModelsDir() {
        return modelsDir;
    }

    public String getOutputDir() {
        return outputDir;
    }

    public String getRootModel() {
        return rootModel;
    }

    public Set<String> getGenerators() {
        return generators;
    }

    public String getEmadlBackend() {
        return emadlBackend;
    }
}
