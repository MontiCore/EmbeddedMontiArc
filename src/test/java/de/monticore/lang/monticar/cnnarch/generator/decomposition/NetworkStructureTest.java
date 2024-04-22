package de.monticore.lang.monticar.cnnarch.generator.decomposition;
import de.monticore.lang.monticar.cnnarch.generator.AbstractSymtabTest;
import org.junit.Test;
import static org.junit.Assert.*;
import de.monticore.symboltable.Scope;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.emadl.modularcnn.compositions.NetworkStructureInformation;

import java.util.List;

public class NetworkStructureTest extends AbstractSymtabTest {

    @Test
    public void testNetworkStructureProcessing(){
        CNNArchCompilationUnitSymbol comp = createModel("src/test/resources/architectures","ResNet");
        String jsonInformation = "{\"name\":\"ResNet\",\"instanceSymbolName\":\"resnet\",\"atomic\":true,\"dataFlow\":[],\"inputDimensions\":{\"data\":[1,28,28]},\"subNetworks\":[]}";
        NetworkStructureInformation networkStructureInformation = new NetworkStructureInformation(jsonInformation);
        NetworkStructure networkStructure = new NetworkStructure(networkStructureInformation, comp.getArchitecture());
        List<LayerInformation> networkLayers = networkStructure.getNetworkLayers();
        assertEquals(56, networkLayers.size());
        assertEquals("data", networkLayers.get(0).getLayerName());
        assertEquals("vector", networkLayers.get(networkLayers.size()-1).getLayerName());
    }
    public CNNArchCompilationUnitSymbol createModel(String modelPath, String model) {
        Scope symTab = createSymTab(modelPath);
        CNNArchCompilationUnitSymbol comp = symTab.<CNNArchCompilationUnitSymbol> resolve(
                model, CNNArchCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull("Could not resolve model " + model, comp);
        return comp;

    }

}
