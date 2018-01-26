package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.PortNameHelper;

public class DirectMsgConverter implements MsgConverter {
    private String msgField;
    private boolean isMsgToPort;

    public DirectMsgConverter(String msgField, boolean isMsgToPort) {
        this.isMsgToPort = isMsgToPort;
        this.msgField = msgField;
    }

    @Override
    public boolean isMsgToPort() {
        return isMsgToPort;
    }

    @Override
    public String getConversion(PortSymbol portSymbol) {
        return !isMsgToPort ? "." + msgField + " = component." + PortNameHelper.getPortNameTargetLanguage(portSymbol) : "msg->" + msgField;
    }


}
