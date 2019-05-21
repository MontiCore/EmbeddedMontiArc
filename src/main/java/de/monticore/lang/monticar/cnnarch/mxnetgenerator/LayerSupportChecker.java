package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerDeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class LayerSupportChecker {

    protected List<String> supportedLayerList = new ArrayList<>();

    public LayerSupportChecker() {
        supportedLayerList.add(AllPredefinedLayers.FULLY_CONNECTED_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONVOLUTION_NAME);
        supportedLayerList.add(AllPredefinedLayers.SOFTMAX_NAME);
        supportedLayerList.add(AllPredefinedLayers.SIGMOID_NAME);
        supportedLayerList.add(AllPredefinedLayers.TANH_NAME);
        supportedLayerList.add(AllPredefinedLayers.RELU_NAME);
        supportedLayerList.add(AllPredefinedLayers.DROPOUT_NAME);
        supportedLayerList.add(AllPredefinedLayers.POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.GLOBAL_POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.LRN_NAME);
        supportedLayerList.add(AllPredefinedLayers.BATCHNORM_NAME);
        supportedLayerList.add(AllPredefinedLayers.SPLIT_NAME);
        supportedLayerList.add(AllPredefinedLayers.GET_NAME);
        supportedLayerList.add(AllPredefinedLayers.ADD_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONCATENATE_NAME);
        supportedLayerList.add(AllPredefinedLayers.FLATTEN_NAME);
    }

    private boolean isSupportedLayer(ArchitectureElementSymbol element){
        ArchitectureElementSymbol resolvedElement = element.getResolvedThis().get();
        List<ArchitectureElementSymbol> constructLayerElemList;

        if (resolvedElement instanceof CompositeElementSymbol) {
            constructLayerElemList = ((CompositeElementSymbol) resolvedElement).getElements();
            for (ArchitectureElementSymbol constructedLayerElement : constructLayerElemList) {
                if (!isSupportedLayer(constructedLayerElement)) {
                    return false;
                }
            }
            return true;
        }

        // Support all inputs and outputs
        if (resolvedElement.isInput() || resolvedElement.isOutput()) {
            return true;
        }

        // Support all layer declarations
        if (resolvedElement instanceof LayerSymbol) {
            if (!((LayerSymbol) resolvedElement).getDeclaration().isPredefined()) {
                return true;
            }
        }

        if (!supportedLayerList.contains(element.toString())) {
            Log.error("Unsupported layer " + "'" + element.getName() + "'" + " for the current backend.");
            return false;
        } else {
            return true;
        }
    }

    public boolean check(ArchitectureSymbol architecture) {
        for (CompositeElementSymbol stream : architecture.getStreams()) {
            for (ArchitectureElementSymbol element : stream.getElements()) {
                if (!isSupportedLayer(element)) {
                    return false;
                }
            }
        }

        return true;
    }

}
