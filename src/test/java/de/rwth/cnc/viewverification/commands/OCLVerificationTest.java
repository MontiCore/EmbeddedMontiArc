/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import de.monticore.lang.ocl.nfp.EMANFPWitness;
import de.rwth.cnc.LogConfig;
import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.BeforeClass;
import org.junit.Test;

import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Created by yannickdeuster on 22.10.17.
 */
public class OCLVerificationTest {
    private static final String TESTDIR = "src/test/resources/";
    private static final String workingDir = Paths.get(".").toAbsolutePath().normalize().toString();
    private static final String nfpDir = workingDir + "/src/test/resources/nfp/";
    private static final String target = workingDir + "/target/gen/";

    @BeforeClass
    public static void init(){
        LogConfig.init();
    }

    public HashMap<String, EMANFPWitness> testOCL(String modelCmp, String viewCmp, String nfpOcl, Boolean expected) {
        Boolean res = ViewVerificator.verifyOCL(nfpDir, modelCmp, viewCmp, nfpOcl, target);
        assertEquals(expected, res);
        return ViewVerificator.getNFPWitnessMap();
    }

    @Test
    public void oclVerificationTest(){
        HashMap<String, EMANFPWitness> witnesses;
        witnesses = testOCL("model.weatherBalloon.WeatherBalloonSensors", "view.wbView.WCET1", "tagDef.semantic.WCET", true);

        assertTrue(!witnesses.isEmpty());

        EMANFPWitness singleWitness = witnesses.get("controlSignalsIndataSaveInternalOutweatherBalloonSensors");
//        System.out.println(singleWitness);
    }

    @Test
    public void verifyViewAndOCLTest(){
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(nfpDir, "model.weatherBalloon.WeatherBalloonSensors", nfpDir, "view.wbView.WCET1", "tagDef.semantic.WCET", target);

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    }
}
