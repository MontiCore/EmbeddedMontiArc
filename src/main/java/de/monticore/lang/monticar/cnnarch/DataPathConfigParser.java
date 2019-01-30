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
package de.monticore.lang.monticar.cnnarch;

import java.io.*;
import java.util.*;
import com.google.common.base.Splitter;

public class DataPathConfigParser {

    public static String getDataPath(String configPath, String modelName){
        
        BufferedReader reader;
        try{
            reader = new BufferedReader(new FileReader(configPath));
            
            String line = reader.readLine();
            List<String> lineList;
        
            while(line != null){
                lineList = Splitter.on(' ').splitToList(line);
                if((lineList.get(0)).equals(modelName)){  
                    List<String> subList = lineList.subList(1,lineList.size());         
                    String path = String.join(" ", subList);
                    reader.close();

                    return path;
                } 
                line = reader.readLine();
                
                
            }
            reader.close();
        } catch(IOException e){
            e.printStackTrace();
        }
        return "Path not found";
    }
}