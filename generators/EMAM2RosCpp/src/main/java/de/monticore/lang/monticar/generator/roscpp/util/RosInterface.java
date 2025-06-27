/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.rosmsg.GeneratorRosMsg;
import de.monticore.lang.monticar.generator.rosmsg.RosField;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;
import java.util.stream.Collectors;

public abstract class RosInterface {
    protected EMAPortSymbol port;
    protected RosConnectionSymbol rosConnectionSymbol;

    public RosMsg getRosMsg() {
        String packageName = Arrays.stream(getTopicType().split("/")).findFirst().get();
        RosMsg res = GeneratorRosMsg.getRosType(packageName, this.getPort().getTypeReference(), false);
        return res;
    }

    public RosMsg getRos2Msg() {
        String packageName = Arrays.stream(getTopicType().split("/")).findFirst().get();
        return GeneratorRosMsg.getRosType(packageName, this.getPort().getTypeReference(), true);
    }

    public String getTopicName(){
        return rosConnectionSymbol.getTopicName().get();
    }

    public EMAPortSymbol getPort() {
        return port;
    }

    public String getPortNameInTargetLanguage(){
        return NameHelper.getPortNameTargetLanguage(getPort());
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

    public abstract String getRosSetStructInstruction();

    public abstract String getRos2SetStructInstruction();

    public abstract String getRosSetMatrixInstruction();

    public abstract String getRos2SetMatrixInstruction();

    public String getTopicType() {
        return getRosConnectionSymbol().getTopicType().get();
    }

    public String getRosInclude(){
        return getTopicType() + ".h";
    }

    public String getRos2Include(){
        return NameHelper.msgTypeToSnakecase(getTopicType()) + ".hpp";
    }

    public boolean isStructInterface(){
        if(!port.getTypeReference().existsReferencedSymbol()){
            return false;
        }

        return port.getTypeReference().getReferencedSymbol() instanceof StructSymbol;
    }

    public boolean isMatrixInterface(){

        if(!port.getTypeReference().existsReferencedSymbol()){
            return false;
        }

        return port.getTypeReference().toString().equals("CommonMatrixType");
    }

}
