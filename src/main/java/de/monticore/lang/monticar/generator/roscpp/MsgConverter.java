package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;

import java.util.Collections;
import java.util.Set;

//TODO: rename
public interface MsgConverter {

    boolean isMsgToPort();

    default boolean isPortToMsg() {
        return !isMsgToPort();
    }

    String getConversion(PortSymbol portSymbol);

    default Set<String> getAdditionalIncludes() {
        return Collections.emptySet();
    }
}
