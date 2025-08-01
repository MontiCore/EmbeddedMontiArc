/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

import org.apache.commons.lang3.SystemUtils;

public class OSHelper {
    private OSHelper(){
    }

    public static String getDirPrefix(){
        if(SystemUtils.IS_OS_WINDOWS){
            return "N:";
        }else{
            return "./";
        }
    }
}
