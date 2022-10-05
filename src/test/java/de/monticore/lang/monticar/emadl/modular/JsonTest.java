/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modular;

import de.monticore.lang.monticar.emadl.modularcnn.composer.NetworkStructureInformation;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static junit.framework.Assert.assertTrue;
import static junit.framework.TestCase.assertEquals;

public class JsonTest {


    @Before
    public void setUp() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void TestJsonHandlingSimple(){
        NetworkStructureInformation net3 = new NetworkStructureInformation("Net3", "net3",true,null,null);
        NetworkStructureInformation net2 = new NetworkStructureInformation("Net2", "net2",true,null,null);
        ArrayList<NetworkStructureInformation> net1Subnets = new ArrayList<>();
        net1Subnets.add(net2);
        net1Subnets.add(net3);
        NetworkStructureInformation net1 = new NetworkStructureInformation("Net1", "net1",false, net1Subnets,null);


        String json = net3.printStructureJSON();
        NetworkStructureInformation net3Read = new NetworkStructureInformation(json);
        assertTrue(net3.equals(net3Read));

        json = net2.printStructureJSON();
        NetworkStructureInformation net2Read = new NetworkStructureInformation(json);
        assertTrue(net2.equals(net2Read));

        json = net1.printStructureJSON();
        NetworkStructureInformation net1Read = new NetworkStructureInformation(json);

        Log.info("JSON","JSON");
        assertTrue(net1.equals(net1Read));
    }

    @Test
    public void TestJsonHandlingComplex(){

        NetworkStructureInformation net5 = new NetworkStructureInformation("Net5", "net5",true,null,null);
        NetworkStructureInformation net4 = new NetworkStructureInformation("Net4", "net4",true,null,null);
        ArrayList<NetworkStructureInformation> net3Subnets = new ArrayList<>();
        net3Subnets.add(net4);
        net3Subnets.add(net5);
        NetworkStructureInformation net3 = new NetworkStructureInformation("Net3", "net3",false,net3Subnets,null);


        NetworkStructureInformation net2 = new NetworkStructureInformation("Net2", "net2",true,null,null);


        ArrayList<NetworkStructureInformation> net1Subnets = new ArrayList<>();
        net1Subnets.add(net2);
        net1Subnets.add(net3);
        NetworkStructureInformation net1 = new NetworkStructureInformation("Net1", "net1",false, net1Subnets,null);

        String json = net5.printStructureJSON();
        NetworkStructureInformation net5Read = new NetworkStructureInformation(json);
        assertTrue(net5.equals(net5Read,true));

        json = net4.printStructureJSON();
        NetworkStructureInformation net4Read = new NetworkStructureInformation(json);
        assertTrue(net4.equals(net4Read,true));

        json = net3.printStructureJSON();
        NetworkStructureInformation net3Read = new NetworkStructureInformation(json);
        assertTrue(net3.equals(net3Read,true));

        json = net2.printStructureJSON();
        NetworkStructureInformation net2Read = new NetworkStructureInformation(json);
        assertTrue(net2.equals(net2Read,true));

        json = net1.printStructureJSON();
        NetworkStructureInformation net1Read = new NetworkStructureInformation(json);

        Log.info("JSON","JSON");
        assertTrue(net1.equals(net1Read));
        assertTrue(net1.isRoot());
    }

    @Test
    public void TestJsonSubnetSearch(){

        NetworkStructureInformation net5 = new NetworkStructureInformation("Net5", "net5",true,null,null);
        NetworkStructureInformation net4 = new NetworkStructureInformation("Net4", "net4",true,null,null);
        ArrayList<NetworkStructureInformation> net3Subnets = new ArrayList<>();
        net3Subnets.add(net4);
        net3Subnets.add(net5);
        NetworkStructureInformation net3 = new NetworkStructureInformation("Net3", "net3",false, net3Subnets,null);


        NetworkStructureInformation net2 = new NetworkStructureInformation("Net2", "net2",true,null,null);


        ArrayList<NetworkStructureInformation> net1Subnets = new ArrayList<>();
        net1Subnets.add(net2);
        net1Subnets.add(net3);
        NetworkStructureInformation net1 = new NetworkStructureInformation("Net1", "net1",false, net1Subnets,null);

        assertTrue(net1.isSubnet("Net5","net5"));
        assertTrue(!net1.isComposedNet("Net5","net5"));

        assertTrue(net1.isSubnet("Net3","net3"));
        assertTrue(net1.isComposedNet("Net3","net3"));

        assertTrue(net1.isComposedNet("Net1","net1"));

    }


}
