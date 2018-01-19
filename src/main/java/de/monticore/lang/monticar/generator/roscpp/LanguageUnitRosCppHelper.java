package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.montiarc.montiarc._symboltable.ExpandedComponentInstanceKind;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.LanguageUnit;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.symboltable.Symbol;

public class LanguageUnitRosCppHelper extends LanguageUnit {
    @Override
    public void generateBluePrints() {
        if (symbolsToConvert.size() > 1)
            throw new IllegalArgumentException("DataHelper does not work for multiple symbols yet!");

        for (Symbol s : symbolsToConvert) {
            if (s.isKindOf(ExpandedComponentInstanceKind.KIND)) {
                this.bluePrints.add(generateHelperBluePrint((ExpandedComponentInstanceSymbol) s));
            }
        }
    }

    private BluePrint generateHelperBluePrint(ExpandedComponentInstanceSymbol s) {
        //TODO: real name
        BluePrint currentBluePrint = new BluePrint("MsgPortHelper");

        for (PortSymbol portSymbol : DataHelper.getPorts()) {
            Method tmpMethod = MethodHelper.getMsgPortConverterMethod(portSymbol, DataHelper.getTopicFromPort(portSymbol).orElse(null));
            tmpMethod.setReturnTypeName("static " + tmpMethod.getReturnTypeName());
            currentBluePrint.addMethod(tmpMethod);
        }

        return currentBluePrint;
    }
}
