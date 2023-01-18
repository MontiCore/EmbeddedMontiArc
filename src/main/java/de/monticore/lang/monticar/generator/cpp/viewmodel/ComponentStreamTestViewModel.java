/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import java.util.Collections;
import java.util.List;

public final class ComponentStreamTestViewModel extends ViewModelBase {

    private String fileNameWithoutExtension;
    private String componentName;
    private List<StreamViewModel> streams = Collections.emptyList();
    private boolean useOpenCV = false;

    public String getFileNameWithExtension() {
        return getFileNameWithoutExtension() + ".hpp";
    }

    public String getFileNameWithoutExtension() {
        return fileNameWithoutExtension;
    }

    public void setFileNameWithoutExtension(String fileNameWithoutExtension) {
        this.fileNameWithoutExtension = fileNameWithoutExtension;
    }

    public String getComponentName() {
        return componentName;
    }

    public void setComponentName(String componentName) {
        this.componentName = componentName;
    }

    public List<StreamViewModel> getStreams() {
        return streams;
    }

    public void setStreams(List<StreamViewModel> streams) {
        this.streams = streams;
    }

    public boolean isUseOpenCV() {
        return useOpenCV;
    }

    public void setUseOpenCV(boolean useOpenCV) {
        this.useOpenCV = useOpenCV;
    }
}
