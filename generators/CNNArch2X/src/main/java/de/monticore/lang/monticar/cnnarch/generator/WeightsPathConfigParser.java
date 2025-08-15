/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.se_rwth.commons.logging.Log;

import java.io.*;
import java.util.Properties;

public class WeightsPathConfigParser{

	private String configTargetPath;
	private Properties properties;

    public WeightsPathConfigParser(String configPath) {
		setConfigPath(configPath);
		properties = new Properties();
		try
		{
			properties.load(new FileInputStream(configTargetPath));
		} catch(IOException e)
		{
			Log.error("Config file " + configPath + " could not be found");
		}
    }

	public String getConfigPath() {
        if (configTargetPath.charAt(configTargetPath.length() - 1) != '/') {
            this.configTargetPath = configTargetPath + "/";
        }
        return configTargetPath;
    }

	public void setConfigPath(String configTargetPath){
		this.configTargetPath = configTargetPath;
	}

	public String getWeightsPath(String modelName) {
		String path = properties.getProperty(modelName);
		if(path == null) {
			Log.info("Weights path config file did not specify a path for component '" + modelName + "'", "WeightsPathConfigParser");
			return path;
		}
		return path;
    }
}
