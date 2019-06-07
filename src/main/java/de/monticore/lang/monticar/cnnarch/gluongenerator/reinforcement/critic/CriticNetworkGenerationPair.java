package de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.critic;

import java.util.Map;

public class CriticNetworkGenerationPair {
    private String criticNetworkName;
    private Map<String, String> fileContent;

    public CriticNetworkGenerationPair(String criticNetworkName, Map<String, String> fileContent) {
        this.criticNetworkName = criticNetworkName;
        this.fileContent = fileContent;
    }

    public String getCriticNetworkName() {
        return criticNetworkName;
    }

    public Map<String, String> getFileContent() {
        return fileContent;
    }
}
