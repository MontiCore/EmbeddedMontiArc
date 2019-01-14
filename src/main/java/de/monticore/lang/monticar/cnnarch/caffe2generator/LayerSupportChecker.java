package de.monticore.lang.monticar.cnnarch.caffe2generator;

import static de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers.*;
import java.util.ArrayList;
import java.util.List;


public class LayerSupportChecker {

    private List<String> unsupportedLayerList = new ArrayList();

    public LayerSupportChecker() {
        //Set the unsupported layers for the backend
        this.unsupportedLayerList.add(ADD_NAME);
        this.unsupportedLayerList.add(SPLIT_NAME);
        this.unsupportedLayerList.add(GET_NAME);
        this.unsupportedLayerList.add(CONCATENATE_NAME);
    }

    public boolean isSupported(String element) {
        return !this.unsupportedLayerList.contains(element);
    }
}
