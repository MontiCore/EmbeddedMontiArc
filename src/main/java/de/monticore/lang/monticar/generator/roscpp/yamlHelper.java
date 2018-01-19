package de.monticore.lang.monticar.generator.roscpp;

import de.monticar.lang.monticar.generator.python.RosInterface;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticore.lang.montiarc.montiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.montiarc.montiarc._symboltable.PortSymbol;
import de.monticore.symboltable.Scope;

public class yamlHelper {

    public static void rosTagToDataHelper(Scope symtab, RosTag rosTag) {
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve(rosTag.component, ExpandedComponentInstanceSymbol.KIND).orElse(null);

        for (RosInterface sub : rosTag.subscriber) {
            for (String portName : sub.ports.keySet()) {
                PortSymbol currentPort = componentInstanceSymbol.getPort(portName).orElse(null);

            }
        }

    }

}
