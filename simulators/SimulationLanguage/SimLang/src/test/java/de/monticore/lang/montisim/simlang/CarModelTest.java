/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang.adapter.SimLangContainer;
import de.monticore.lang.montisim.util.types.ExplicitVehicle;
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

        for (ExplicitVehicle v : carModelTest.getExplicitVehicles().get()) {
            switch (v.getName()) {
                case "car1":
                case "car3":
                    Assert.assertEquals(v.getCarContainer().getMass(), 1800, DELTA);
                    break;
                case "car2":
                    Assert.assertEquals(v.getCarContainer().getMass(), 2800, DELTA);
                    break;
            }
        }
    }

}
