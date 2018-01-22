package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertNotNull;

public class RosTargetTest extends AbstractSymtabTest {

    @Test
    public void rosTargetTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/testRos/");

        //in:/sim/objects_ground_truth
        PortSymbol portRosIn = componentSymbol.getIncomingPort("rosIn").orElse(null);
        RosTopic groundTruthTopic = new RosTopic("/sim/objects_ground_truth", "ObjectStateArray", "automated_driving_msgs/ObjectStateArray");

        //out:/sim/desired_motion or /echo
        PortSymbol portRosOut = componentSymbol.getOutgoingPort("rosOut").orElse(null);
        RosTopic echoTopic = new RosTopic("/echo", "StampedFloat64", "automated_driving_msgs/StampedFloat64");

        DataHelper.addPortToTopic(portRosIn, groundTruthTopic, "test");
        DataHelper.addPortToTopic(portRosOut, echoTopic, "test2");


        generatorRosCpp.generateFiles(componentSymbol, symtab);

        //TODO: compare result
    }

}
