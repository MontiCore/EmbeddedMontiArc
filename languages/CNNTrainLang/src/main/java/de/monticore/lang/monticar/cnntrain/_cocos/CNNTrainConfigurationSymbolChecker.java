/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class CNNTrainConfigurationSymbolChecker {
    private List<CNNTrainConfigurationSymbolCoCo> cocos = new ArrayList<>();

    public CNNTrainConfigurationSymbolChecker addCoCo(CNNTrainConfigurationSymbolCoCo coco) {
        cocos.add(coco);
        return this;
    }

    public void checkAll(ConfigurationSymbol configurationSymbol) {
        for (CNNTrainConfigurationSymbolCoCo coco : cocos) {
            coco.check(configurationSymbol);
        }
    }
}