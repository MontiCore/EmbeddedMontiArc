/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.instructions.SetStructPortInstruction;
import de.se_rwth.commons.logging.Log;

public class RosSubscriber extends RosInterface {

    public RosSubscriber(EMAPortSymbol port) {
        if (port.isOutgoing()) {
            Log.error("Subscriber can only be created from a incoming Port but " + port.getFullName() + " is outgoing!");
        }
        if (!port.isRosPort()) {
            Log.error("A RosSubscriber can only be created from a ros Ports but " + port.getFullName() + " is not one!");
        }
        this.port = port;
        this.rosConnectionSymbol = (RosConnectionSymbol) port.getMiddlewareSymbol().get();
    }

    @Override
    public String getNameInTargetLanguage() {
        return NameHelper.getTopicNameTargetLanguage(rosConnectionSymbol.getTopicName().get()).toLowerCase() + "Subscriber";
    }

    @Override
    public String getMethodName() {
        return NameHelper.getTopicNameTargetLanguage(rosConnectionSymbol.getTopicName().get()).toLowerCase() + "Callback";
    }

    @Override
    public String getRosSetStructInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        return SetStructPortInstruction.getStructInstruction(getPort(), getRosMsg(), fieldPrefix);
    }
 
    @Override
    public String getRosSetMatrixInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        return SetStructPortInstruction.getMatrixInstruction(getPort(), getRosMsg(), fieldPrefix);
    }

    @Override
    public String getRos2SetStructInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        return SetStructPortInstruction.getStructInstruction(getPort(), getRos2Msg(),fieldPrefix);
    }

    @Override
    public String getRos2SetMatrixInstruction() {
        String fieldPrefix = this.rosConnectionSymbol.getMsgField().map(msgfield -> msgfield + ".").orElse("");
        return SetStructPortInstruction.getMatrixInstruction(getPort(), getRos2Msg(),fieldPrefix);
    }
}
