/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.tools;

import java.text.SimpleDateFormat;
import java.util.Calendar;

public class Randomizer {

    public static String timeStamp(){
        String dateFormat = "yyyy-MM-dd_HH-mm-ss";
        Calendar calendar = Calendar.getInstance();
        SimpleDateFormat simpleDateFormat = new SimpleDateFormat(dateFormat);
        return "_" + simpleDateFormat.format(calendar.getTime());
    }
}
