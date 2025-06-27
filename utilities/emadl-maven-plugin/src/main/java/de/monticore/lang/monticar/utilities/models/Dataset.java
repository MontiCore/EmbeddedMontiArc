package de.monticore.lang.monticar.utilities.models;

import java.io.File;

public class Dataset {
    private String id;

    private String references;

    private String path;

    private String description;

    private String filetype;

    public String getFiletype() {
        return filetype;
    }

    public void setFiletype(String filetype) {
        this.filetype = filetype;
    }

    public String getGraphFilePath() {
        return graphFilePath;
    }

    public void setGraphFilePath(String graphFilePath) {
        this.graphFilePath = graphFilePath;
    }

    public boolean isTesting() {
        return testing;
    }

    public void setTesting(boolean testing) {
        this.testing = testing;
    }

    private String graphFilePath;

    private boolean testing;

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public String getReferences() {
        return references;
    }

    public void setReferences(String references) {
        this.references = references;
    }

    public String getPath() {
        return path;
    }

    public void setPath(String path) {
        this.path = path;
    }

    public String getDescription() {
        return description;
    }

    public void setDescription(String description) {
        this.description = description;
    }
}
