/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArgumentSymbol;

import de.se_rwth.commons.logging.Log;
import java.io.File;
import java.util.*;

public class CheckLayerPathParameter {

    public static void check(LayerSymbol sym, String path, String tag, HashMap layerPathParameterTags) {
        checkTag(sym, tag, layerPathParameterTags);
        checkPath(sym, path);
    }

    protected static void checkTag(LayerSymbol layer, String tag, HashMap layerPathParameterTags){
        if (!tag.equals("") && !layerPathParameterTags.containsKey(tag)) {
            Log.error("0" + ErrorCodes.INVALID_LAYER_PATH_PARAMETER_TAG +
                            "The LayerPathParameter tag " + tag + " was not found.",
                    layer.getSourcePosition());
        }
    }

    protected static void checkPath(LayerSymbol layer, String path) {
        File dir = new File(path);
        if (dir.exists()) {
            return;
        }
        Log.error("0" + ErrorCodes.INVALID_LAYER_PATH_PARAMETER_PATH +
                        " For the concatination of queryNetDir and queryNetPrefix exists no file which path has this as prefix.",
                layer.getSourcePosition());
    }
}
