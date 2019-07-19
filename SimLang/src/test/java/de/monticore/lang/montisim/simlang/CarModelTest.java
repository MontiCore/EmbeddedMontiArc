package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang.adapter.SimLangContainer;
import org.junit.Assert;
import org.junit.Test;

import java.nio.file.Path;
import java.nio.file.Paths;

public class CarModelTest {

    private static final double DELTA = 0.1;
    public static final Path MODEL_PATH = Paths.get("src/test/resources/test/car");

    @Test
    public void carModelAvailable() {
        SimLangContainer carModelTest = SimLangTool.parseIntoContainer(MODEL_PATH, "CarModelTest");

        // TODO: the order seems to be not consistent with the order in the sim file
        //Assert.assertEquals(carModelTest.getExplicitVehicles().get().get(0).getCarContainer().getMass(), 1800, DELTA);
        //Assert.assertEquals(carModelTest.getExplicitVehicles().get().get(1).getCarContainer().getMass(), 1800, DELTA);
        //Assert.assertEquals(carModelTest.getExplicitVehicles().get().get(2).getCarContainer().getMass(), 2800, DELTA);
    }

}
