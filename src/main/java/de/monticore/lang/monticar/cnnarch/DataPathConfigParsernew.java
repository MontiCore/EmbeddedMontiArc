package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnntrain._symboltable.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DataPathConfigParser  {

    private String configTargetPath;
	ConfigurationSymbol configuration;
    private String configFileName;

    public DataPathConfigParser (ConfigurationSymbol configuration) {
        this.configuration = configuration;
		setConfigPath("./target/"); 
		setConfigFileName("ConfigChainAutomation");
    }	
	
	public String getConfigFileName() {
        return configFileName;
    }
	
	public void setConfigFileName(String configFileName){
		this.configFileName = configFileName + ".cnnt";
	}
	
	public String getConfigPath() {
        if (configTargetPath.charAt(configTargetPath.length() - 1) != '/') {
            this.configTargetPath = configTargetPath + "/";
        }
        return configTargetPath;
    }
	
	public void setConfigPath(String configTargetPath){
		this.configTargetPath = configTargetPath + getConfigFileName;
	}
	
    public ConfigurationSymbol getConfiguration() {
        return configuration;
    }

	public String getDataPath(String componentName) {
        if (!getConfiguration().getEntryMap().containsKey(componentName)) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry(componentName).getValue());
    }
}