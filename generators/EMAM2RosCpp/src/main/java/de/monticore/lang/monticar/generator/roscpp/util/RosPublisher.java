/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.instructions.SetStructMsgInstruction;
import de.se_rwth.commons.logging.Log;

public class RosPublisher extends RosInterface {

    public RosPublisher(EMAPortSymbol port) {
        if (port.isIncoming()) {
            Log.error("Publisher can only be created from a outgoing Port but " + port.getFullName() + " is incoming!");
        }
        if (!port.isRosPort()) {
            Log.error("A RosSubscriber can only be created from a ros Ports but " + port.getFullName() + " is not one!");
        }
        this.port = port;
        this.rosConnectionSymbol = (RosConnectionSymbol) port.getMiddlewareSymbol().get();
    }

    @Override
    public String getNameInTargetLanguage() {
        return NameHelper.getTopicNameTargetLanguage(rosConnectionSymbol.getTopicName().get()).toLowerCase() + "Publisher";
    }

    @Override
    public String getMethodName() {
        return "publish" + getNameInTargetLanguage();
    }

    @Override
    public String getRosSetStructInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        //return SetStructMsgInstruction.getStructInstruction(getPort(), getRosMsg(), fieldPrefix);
        return SetStructMsgInstruction.getInstruction(getPort(), getRosMsg(), fieldPrefix);
    }
 
    @Override
    public String getRosSetMatrixInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        //return SetStructMsgInstruction.getMatrixInstruction(getPort(), getRosMsg(), fieldPrefix);
        return SetStructMsgInstruction.getInstruction(getPort(), getRosMsg(), fieldPrefix);
    }

    @Override
    public String getRos2SetStructInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        //return SetStructMsgInstruction.getStructInstruction(getPort(), getRos2Msg(),fieldPrefix);
        return SetStructMsgInstruction.getInstruction(getPort(), getRosMsg(), fieldPrefix);
    }

    @Override
    public String getRos2SetMatrixInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        //return SetStructMsgInstruction.getMatrixInstruction(getPort(), getRos2Msg(),fieldPrefix);
        return SetStructMsgInstruction.getInstruction(getPort(), getRosMsg(), fieldPrefix);
    }
}
