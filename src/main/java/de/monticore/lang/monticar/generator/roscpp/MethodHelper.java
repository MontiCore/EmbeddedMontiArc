package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;

public class MethodHelper {
    public static Method getMsgPortConverterMethod(PortSymbol portSymbol, RosTopic rosTopic) {
        Method res = new Method();

        Variable variable = new Variable();
        String typeNameMontiCar = portSymbol.getTypeReference().getName();

        variable.setInputVariable(portSymbol.isIncoming());

        VariableType emamTypeString = TypeConverter.getVariableTypeForMontiCarTypeName(typeNameMontiCar, variable, portSymbol).get();

        String parameterTypeString;

        if (portSymbol.isIncoming()) {
            res.setName("get" + getPortNameTargetLanguage(portSymbol) + "From" + rosTopic.getTargetLanguageName());
            res.setReturnTypeName(emamTypeString.getTypeNameTargetLanguage());
            parameterTypeString = "const " + rosTopic.getFullRosType() + "::ConstPtr&";
        } else {
            res.setName("get" + rosTopic.getTargetLanguageName() + "From" + getPortNameTargetLanguage(portSymbol));
            res.setReturnTypeName(rosTopic.getFullRosType());
            parameterTypeString = emamTypeString.getTypeNameTargetLanguage();
        }

        Variable var = new Variable();
        var.setName("var1");
        var.setTypeNameTargetLanguage(parameterTypeString);
        res.addParameter(var);

        return res;
    }

    //TODO: refactor
    public static String getPortNameTargetLanguage(PortSymbol portSymbol) {
        //TODO: arrays
        //TODO: check format
        //TODO: from cpp generator?
        return portSymbol.getName();
    }
}
