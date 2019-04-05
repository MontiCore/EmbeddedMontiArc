package de.monticore.lang.monticar.generator.roscpp.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.DirectMsgConverter;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.GeneratorRosMsg;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;

import java.util.Arrays;

public abstract class RosInterface {
    protected EMAPortSymbol port;
    protected RosConnectionSymbol rosConnectionSymbol;

    public RosMsg getRosMsg() {
        String packageName = Arrays.stream(this.getRosConnectionSymbol().getTopicType().get().split("/")).findFirst().get();
        return GeneratorRosMsg.getRosType(packageName, this.getPort().getTypeReference(), false);
    }

    public RosMsg getRos2Msg() {
        String packageName = Arrays.stream(this.getRosConnectionSymbol().getTopicType().get().split("/")).findFirst().get();
        return GeneratorRosMsg.getRosType(packageName, this.getPort().getTypeReference(), true);
    }

    public DirectMsgConverter getMsgConverter() {
        DirectMsgConverter tmpMsgConverter;
        tmpMsgConverter = new DirectMsgConverter(getRosConnectionSymbol().getMsgField().get(), getPort().isIncoming());
        return tmpMsgConverter;
    }

    public EMAPortSymbol getPort() {
        return port;
    }

    public RosConnectionSymbol getRosConnectionSymbol() {
        return rosConnectionSymbol;
    }

    public abstract String getNameInTargetLanguage();

    public abstract String getMethodName();

    public String getTopicNameInTargetLanguage() {
        return NameHelper.getTopicNameTargetLanguage(rosConnectionSymbol.getTopicName().get());
    }

    public String getTypeNameInTargetLanguage() {
        return NameHelper.getFullRosType(rosConnectionSymbol);
    }
}
