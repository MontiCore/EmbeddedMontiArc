package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;

import java.util.Collections;
import java.util.Set;

public class DirectMsgConverter {
    private String msgField;
    private boolean isMsgToPort;

    public DirectMsgConverter(String msgField, boolean isMsgToPort) {
        this.isMsgToPort = isMsgToPort;
        this.msgField = msgField;
    }

    public boolean isMsgToPort() {
        return isMsgToPort;
    }

    public String getConversion(EMAPortSymbol portSymbol) {
        return !isMsgToPort ? "." + msgField + " = component->" + NameHelper.getPortNameTargetLanguage(portSymbol) : "msg->" + msgField;
    }


    public boolean isPortToMsg() {
        return !isMsgToPort();
    }

    public Set<String> getAdditionalIncludes() {
        return Collections.emptySet();
    }
}
