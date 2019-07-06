/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnnarch.generator;

import de.se_rwth.commons.logging.Log;

import java.io.*;
import java.net.URL;
import java.util.Objects;
import java.util.Properties;

public class DataPathConfigParser{
	
	private String configTargetPath;
	private String configFileName;
	private Properties properties;

    public DataPathConfigParser(String configPath) {
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

	public String getDataPath(String modelName) {
		String path = properties.getProperty(modelName);
		if(path == null) {
			Log.error("Data path config file did not specify a path for component '" + modelName + "'");
		}
		return path;
    }
}
