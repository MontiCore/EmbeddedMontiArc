package de.monticore.lang.monticar.generator.middleware;

import java.util.Set;

public class CliParameters {
    private static final boolean DEFAULT_WRITE_TAG_FILE = false;
    private static final String DEFAULT_EMADL_BACKEND = "MXNET";

    private String modelsDir;
    private String outputDir;
    private String rootModel;
    private Set<String> generators;
    private String emadlBackend;
    private Boolean writeTagFile;

    public CliParameters() {
    }

    public CliParameters(String modelsDir, String outputDir, String rootModel, Set<String> generators, String emadlBackend, Boolean writeTagFile) {
        this.modelsDir = modelsDir;
        this.outputDir = outputDir;
        this.rootModel = rootModel;
        this.generators = generators;
        this.emadlBackend = emadlBackend == null ? DEFAULT_EMADL_BACKEND : emadlBackend;
        this.writeTagFile = writeTagFile == null ? DEFAULT_WRITE_TAG_FILE : writeTagFile;
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

    public boolean getWriteTagFile() {
        return writeTagFile;
    }
}
