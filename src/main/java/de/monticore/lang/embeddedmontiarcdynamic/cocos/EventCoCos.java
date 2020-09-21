/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.cocos;

import de.monticore.lang.embeddedmontiarcdynamic.event._cocos.EventCapitalized;
import de.monticore.lang.embeddedmontiarcdynamic.event._cocos.EventCoCoChecker;

public class EventCoCos {


    public static EventCoCoChecker createChecker(){
        return new EventCoCoChecker()
                .addCoCo(new EventCapitalized());
    }


}
