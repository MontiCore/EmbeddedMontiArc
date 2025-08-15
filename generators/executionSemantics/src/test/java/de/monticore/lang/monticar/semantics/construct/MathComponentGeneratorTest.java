package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.monticar.semantics.Constants;
import org.junit.Test;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class MathComponentGeneratorTest {

    @Test
    public void generate() {
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
                ins, outs, statementList, Constants.SYNTHESIZED_COMPONENTS_ROOT);

    }

}