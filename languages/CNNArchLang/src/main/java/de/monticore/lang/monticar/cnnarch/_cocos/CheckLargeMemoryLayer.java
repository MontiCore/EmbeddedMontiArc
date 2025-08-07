/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArgumentSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;
import java.util.List;


public class CheckLargeMemoryLayer extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureElementSymbol sym) {
        if (sym instanceof LayerSymbol && sym.getName().equals("LargeMemory")) {
            checkLargeMemoryLayer((LayerSymbol) sym);
        }
    }

    protected void checkLargeMemoryLayer(LayerSymbol layer) {
        List<ArgumentSymbol> arguments = layer.getArguments();
        Integer subKeySize = new Integer(0);
        Integer k = new Integer(0);

        for (ArgumentSymbol arg : arguments) {
            if (arg.getName().equals("subKeySize") && arg.getRhs().getIntValue().isPresent()) {
                subKeySize = arg.getRhs().getIntValue().get();
            } else if (arg.getName().equals("k") && arg.getRhs().getIntValue().isPresent()) {
                k = arg.getRhs().getIntValue().get();
            }
        }

        if (subKeySize < k) {
            Log.error("0" + ErrorCodes.INVALID_LARGE_MEMORY_LAYER_PARAMETERS +
                      " Invalid LargeMemory layer Parameter values, subKeySize has to be greater or equal to k. ",
                      layer.getSourcePosition());
        }
    }
}
