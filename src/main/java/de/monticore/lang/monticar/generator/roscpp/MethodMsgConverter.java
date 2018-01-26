package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.collect.Sets;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.PortNameHelper;

import java.util.Set;

//TODO: rename
public class MethodMsgConverter implements MsgConverter {
    private String methodNameWithClass;
    private String includeString;
    private boolean isMsgToPort;


    public MethodMsgConverter(String methodNameWithClass, String includeString, boolean isMsgToPort) {
        this.methodNameWithClass = methodNameWithClass;
        this.includeString = includeString;
        this.isMsgToPort = isMsgToPort;
    }

    @Override
    public boolean isMsgToPort() {
        return isMsgToPort;
    }

    @Override
    public String getConversion(PortSymbol portSymbol) {
        return isMsgToPort ? methodNameWithClass + "(msg)" : " = " + methodNameWithClass + "(component." + (PortNameHelper.getPortNameTargetLanguage(portSymbol) + ")");
    }

    @Override
    public Set<String> getAdditionalIncludes() {
        return Sets.newHashSet(includeString);
    }
}
