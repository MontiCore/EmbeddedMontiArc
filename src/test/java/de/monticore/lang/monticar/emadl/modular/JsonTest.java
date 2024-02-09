/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modular;

import de.monticore.lang.monticar.emadl.modularcnn.compositions.NetworkStructureInformation;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class JsonTest {
    private static final String IMAGE = "image";
    private static final String FLATTENED = "dense";
    private static final String[] NET = {"NET1", "NET2", "NET3", "NET4", "NET5"};
    private static final ArrayList<Integer> DIMENSIONS_IMAGE = new ArrayList<>(Arrays.asList(1, 28, 28));
    private static final ArrayList<Integer> DIMENSIONS_FLATTENED = new ArrayList<>(Arrays.asList(1, 256));
    private final HashMap<String, ArrayList<Integer>> inputPortDimensions = new HashMap<>();

    @Before
    public void setUp() {
        inputPortDimensions.put(IMAGE, DIMENSIONS_IMAGE);
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testJsonHandlingSimple() {
        NetworkStructureInformation network2 = createAtomicNetwork(NET[2]);
        NetworkStructureInformation network1 = createAtomicNetwork(NET[1]);
        NetworkStructureInformation network0 = createParentNetwork(NET[0], new ArrayList<>(Arrays.asList(network1, network2)));
        verifyJsonSerialization(network0);
        verifyJsonSerialization(network1);
        verifyJsonSerialization(network2);
        Log.info("testJsonHandlingSimple","JSON");

    }

    @Test
    public void testJsonHandlingComplex() {
        inputPortDimensions.put(FLATTENED, DIMENSIONS_FLATTENED);
        NetworkStructureInformation network4 = createAtomicNetwork(NET[4]);
        NetworkStructureInformation network3 = createAtomicNetwork(NET[3]);
        NetworkStructureInformation network2 = createParentNetwork(NET[2], new ArrayList<>(Arrays.asList(network3, network4)));
        NetworkStructureInformation network1 = createAtomicNetwork(NET[1]);
        NetworkStructureInformation network0 = createParentNetwork(NET[0], new ArrayList<>(Arrays.asList(network1, network2)));
        verifyJsonSerialization(network0);
        verifyJsonSerialization(network2);
        Log.info("testJsonHandlingComplex","JSON");
    }

    @Test
    public void testJsonSubnetSearch() {
        NetworkStructureInformation network4 = createAtomicNetwork(NET[4]);
        NetworkStructureInformation network3 = createAtomicNetwork(NET[3]);
        NetworkStructureInformation network2 = createParentNetwork(NET[2], new ArrayList<>(Arrays.asList(network3, network4)));
        NetworkStructureInformation network1 = createAtomicNetwork(NET[1]);
        NetworkStructureInformation network0 = createParentNetwork(NET[0], new ArrayList<>(Arrays.asList(network1, network2)));

        assertFalse(network4.isRoot());
        assertFalse(network3.isRoot());
        assertFalse(network2.isRoot());
        assertFalse(network1.isRoot());
        assertTrue("Network0 should be root", network0.isRoot());

        assertTrue(verifySubnet(network2,NET[3]));
        assertTrue(verifySubnet(network2,NET[4]));
        assertTrue(verifySubnet(network0,NET[4]));
        assertTrue(verifySubnet(network0,NET[3]));
        assertTrue(verifySubnet(network0,NET[2]));
        assertTrue(verifySubnet(network0,NET[1]));
        Log.info("testJsonSubnetSearch","JSON");
    }
    @Test
    public void testJsonComposedNet() {
        NetworkStructureInformation network4 = createAtomicNetwork(NET[4]);
        NetworkStructureInformation network3 = createAtomicNetwork(NET[3]);
        NetworkStructureInformation network2 = createParentNetwork(NET[2], new ArrayList<>(Arrays.asList(network3, network4)));
        NetworkStructureInformation network1 = createAtomicNetwork(NET[1]);
        NetworkStructureInformation network0 = createParentNetwork(NET[0], new ArrayList<>(Arrays.asList(network1, network2)));

        assertFalse(verifyComposed(network4, NET[4]));
        assertFalse(verifyComposed(network2,NET[4]));
        assertTrue(verifyComposed(network2,NET[2]));
        assertTrue(verifyComposed(network0,NET[2]));
        assertTrue(verifyComposed(network0,NET[0]));

        assertTrue(verifySubnet(network0,NET[4]));
        assertTrue(verifySubnet(network0, NET[3]));
        assertTrue(verifySubnet(network0, NET[2]));
        assertTrue(verifySubnet(network0, NET[1]));
        Log.info("testJsonComposedNet","JSON");


    }
    private boolean verifyComposed(NetworkStructureInformation network, String networkName){
        return network.isComposedNet(networkName, networkName.toLowerCase());
    }
    private boolean verifySubnet(NetworkStructureInformation network, String networkName){
        return network.isSubnet(networkName, networkName.toLowerCase());
    }

    private void verifyJsonSerialization(NetworkStructureInformation network) {
        String json = network.printStructureJSON();
        NetworkStructureInformation networkRead = new NetworkStructureInformation(json);
        assertTrue("Network should be equal to its JSON representation", network.equals(networkRead, true));
    }
    private NetworkStructureInformation createParentNetwork(String name, ArrayList<NetworkStructureInformation> subNets) {
            return new NetworkStructureInformation(name, name.toLowerCase(), inputPortDimensions, false, subNets, null, null);
    }
    private NetworkStructureInformation createAtomicNetwork(String name) {
        return new NetworkStructureInformation(name, name.toLowerCase(), inputPortDimensions, true, null, null, null);
    }
}
