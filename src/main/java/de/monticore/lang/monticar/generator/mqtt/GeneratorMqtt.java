package de.monticore.lang.monticar.generator.mqtt;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class GeneratorMqtt {

	private String generationTargetPath;
    private boolean generateCMake = false;
    private boolean mqtt2Mode = false;

    public boolean isMqtt2Mode() {
        return mqtt2Mode;
    }
    
    public void setMqtt2Mode(boolean mqtt2Mode) {
        this.mqtt2Mode = mqtt2Mode;
    }

    public void setGenerateCMake(boolean generateCMake) {
        this.generateCMake = generateCMake;
    }

    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath.endsWith("/") ? generationTargetPath : generationTargetPath + "/";
    }

    
    // Those methods use ROS specific fileContent and need to be rewritten
    public List<File> generateFiles(EMAComponentInstanceSymbol component, TaggingResolver symtab) throws IOException {
        return null;
    }
    
    // public List<FileContent> generateStrings(EMAComponentInstanceSymbol component) {}

    // private List<FileContent> generateMqttAdapter(EMAComponentInstanceSymbol component) {}

    // public List<FileContent> generateCMakeFiles(EMAComponentInstanceSymbol component, List<MqttInterface> mqttInterfaces, boolean mqtt2Mode) {}

    
}
