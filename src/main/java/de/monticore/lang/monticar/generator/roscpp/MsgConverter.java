package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;

import java.util.Collections;
import java.util.Set;

//TODO: rename
public interface MsgConverter {

    boolean isMsgToPort();

    default boolean isPortToMsg() {
        return !isMsgToPort();
    }

    String getConversion(EMAPortSymbol portSymbol);

    default Set<String> getAdditionalIncludes() {
        return Collections.emptySet();
    }
}
