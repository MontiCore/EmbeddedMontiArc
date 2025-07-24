/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import java.util.HashSet;
import java.util.Set;

public class AllAttentionModels {

    public static Set<String> getAttentionModels() {
        //List of all models that use attention and should save images of the attention over time
        Set models = new HashSet();
        models.add("showAttendTell.Show_attend_tell");

        return models;
    }

}
