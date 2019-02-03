package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import static de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers.*;
import java.util.ArrayList;
import java.util.List;


public class LayerSupportChecker {

    private List<String> unsupportedLayerList = new ArrayList();

    public LayerSupportChecker() {
        //Set the unsupported layers for the backend
        //this.unsupportedLayerList.add(PREDEFINED_LAYER_NAME);
    }

    public boolean isSupported(String element) {
        return !this.unsupportedLayerList.contains(element);
    }
}
