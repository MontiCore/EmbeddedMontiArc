package de.monticore.lang.monticar.cnnarch.mxnetgenerator.checker;

import de.monticore.lang.monticar.cnnarch.mxnetgenerator.checker.LayerSupportChecker;

import java.util.ArrayList;
import java.util.List;


public class AllowAllLayerSupportChecker implements LayerSupportChecker {

    private List<String> unsupportedLayerList = new ArrayList<>();

    public AllowAllLayerSupportChecker() {
        //Set the unsupported layers for the backend
        //this.unsupportedLayerList.add(PREDEFINED_LAYER_NAME);
    }

    @Override
    public boolean isSupported(String element) {
        return !this.unsupportedLayerList.contains(element);
    }
}
