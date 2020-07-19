/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.StreamInstructionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParallelCompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.SerialCompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArgumentSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;
import java.util.List;
import java.io.File;

public class CheckEpisodicMemoryLayer extends CNNArchSymbolCoCo {

    @Override
    public void check(StreamInstructionSymbol stream) {
        List<ArchitectureElementSymbol> elements = stream.getBody().getElements();

        for (ArchitectureElementSymbol element : elements) {
            if (element instanceof ParallelCompositeElementSymbol) {
                checkForEpisodicMemory((ParallelCompositeElementSymbol) element);
            } else if (element.getName().equals("EpisodicMemory")) {
                checkParameters((LayerSymbol) element);
            }
        }
    }

    public void checkForEpisodicMemory(ParallelCompositeElementSymbol parallelElement) {
        for (ArchitectureElementSymbol subStream : parallelElement.getElements()) {
            if (subStream instanceof SerialCompositeElementSymbol) { //should always be the case
                for (ArchitectureElementSymbol element : ((SerialCompositeElementSymbol) subStream).getElements()) {
                    if (element instanceof ParallelCompositeElementSymbol) {
                        checkForEpisodicMemory((ParallelCompositeElementSymbol) element);
                    } else if (element.getName().equals("EpisodicMemory")) {
                        Log.error("0" + ErrorCodes.INVALID_EPISODIC_MEMORY_LAYER_PLACEMENT +
                                        " Invalid placement of EpisodicMemory layer. It can't be placed inside a Prallalel execution block.",
                                element.getSourcePosition());
                    }
                }
            }
        }
    }

    public void checkParameters(LayerSymbol layer) {
        List<ArgumentSymbol> arguments = layer.getArguments();
        String queryNetDir = new String("");
        String queryNetPrefix = new String("");

        for (ArgumentSymbol arg : arguments) {
            if (arg.getName().equals("queryNetDir")) {
                queryNetDir = arg.getRhs().getStringValue().get();
            } else if (arg.getName().equals("queryNetPrefix")) {
                queryNetPrefix = arg.getRhs().getStringValue().get();
            }
        }

        File dir = new File(queryNetDir);
        if (dir.exists()) {
            for (File file : dir.listFiles()) {
                String file_name = file.getName();
                if (file_name.startsWith(queryNetPrefix)) {
                    return;
                }
            }
        }
        Log.error("0" + ErrorCodes.INVALID_EPISODIC_QUERY_NET_PATH_OR_PREFIX +
                        " For the concatination of queryNetDir and queryNetPrefix exists no file wich path has this as prefix.",
                        layer.getSourcePosition());
    }
}
