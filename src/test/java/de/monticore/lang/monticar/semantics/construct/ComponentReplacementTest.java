/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.symboltable.GlobalScope;
import org.junit.Test;

import java.util.*;

public class ComponentReplacementTest {

    @Test
    public void test() {
        MathComponentGenerator componentCreator = new MathComponentGenerator();
        Map<String, String> ins = new HashMap<>();
        Map<String, String> outs = new HashMap<>();
        ins.put("synthPort_SimpleLoop_in1", "Q");
        ins.put("in1", "Q");
        ins.put("in2", "Q");
        outs.put("out1", "Q");
        List<String> statementList = new LinkedList<>();
        statementList.add("out1 = synthPort_SimpleLoop_in1 / 2");

        componentCreator.generate("DifferenceSynthesized", "de.monticore.lang.monticar.semantics.synth",
                ins, outs, statementList, "target/generated-components");


        ComponentReplacement componentReplacement = new ComponentReplacement("SimpleLoop", "difference",
                "de.monticore.lang.monticar.semantics.synth", "DifferenceSynthesized", "difference");
        Set<ComponentReplacement> componentReplacements = new HashSet<>();
        componentReplacements.add(componentReplacement);
        GlobalScope symTab = SymtabCreator.createSymTabForReplacement(componentReplacements, "src/test/resources", "src/main/resources", "target/generated-components");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("de.monticore.lang.monticar.semantics.loops.SimpleLoop", EMAComponentSymbol.KIND).orElse(null);

        EMAComponentInstanceSymbol emaComponentSymbol = symTab.<EMAComponentInstanceSymbol>resolve("de.monticore.lang.monticar.semantics.loops.simpleLoop.difference", EMAComponentInstanceSymbol.KIND).orElse(null);
        assert ("DifferenceSynthesized difference".equals(emaComponentSymbol.toString()));
    }
}
