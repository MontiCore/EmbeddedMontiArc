package de.monticore.lang.monticar.cnnarch.mxnetgenerator;
package de.monticore.lang.monticar.emadl.generator;

import java.io.*;
import java.net.URL;
import java.util.Objects;
import java.util.Properties;

public class DataPathConfigParser{
	
	private String configTargetPath;
    private String configFileName;

    public DataPathConfigParser(String configPath) {
		setConfigPath(configPath); 
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

	public String getDataPath(String modelName) {
		Properties properties = new Properties();
		
		try
		{
			properties.load(new FileInputStream(configTargetPath));
		}catch(IOException e)
		{
			 e.printStackTrace();
		}
        for (String key: properties.stringPropertyNames())
		{
			if (key == modelName)
				return properties.getProperty(key);
		}
    }
}