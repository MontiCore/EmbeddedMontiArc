/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._cocos;

import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTComponentEvent;
import de.se_rwth.commons.logging.Log;

public class EventCapitalized implements EventASTComponentEventCoCo {

    @Override
    public void check(ASTComponentEvent node) {
        if(!Character.isUpperCase(node.getName().charAt(0))){
            Log.error("0xAC004 Event names must be startVal in upper-case",
                    node.get_SourcePositionStart());
        }
    }
}
