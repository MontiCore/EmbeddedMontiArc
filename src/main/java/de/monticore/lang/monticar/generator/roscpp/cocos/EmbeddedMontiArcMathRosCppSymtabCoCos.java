package de.monticore.lang.monticar.generator.roscpp.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._cocos.EmbeddedMontiArcMathCoCoChecker;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.EmbeddedMontiArcMathCoCos;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class EmbeddedMontiArcMathRosCppSymtabCoCos {
    public static EmbeddedMontiArcMathCoCoChecker createChecker() {
        return EmbeddedMontiArcMathCoCos.createChecker();
    }

    public static EmbeddedMontiArcMathSymtabCoCoChecker createSymtabChecker(TaggingResolver taggingResolver) {
        return new EmbeddedMontiArcMathSymtabCoCoChecker(taggingResolver)
                .addCoCo(new MaxOneRosConnectionPerPort())
                .addCoCo(new InRosPortRosSender());
    }
}
