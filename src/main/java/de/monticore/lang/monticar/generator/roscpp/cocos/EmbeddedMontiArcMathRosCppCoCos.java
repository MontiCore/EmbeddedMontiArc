package de.monticore.lang.monticar.generator.roscpp.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._cocos.EmbeddedMontiArcMathCoCoChecker;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.EmbeddedMontiArcMathCoCos;

public class EmbeddedMontiArcMathRosCppCoCos {
    public static EmbeddedMontiArcMathCoCoChecker createChecker() {
        return EmbeddedMontiArcMathCoCos.createChecker()
                .addCoCo(new InRosPortRosSender());
    }
}
