package de.monticore.lang.monticar.semantics.loops;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.EMAGraphTransformation;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.symboltable.Scope;
import org.junit.Test;

public class TransformGraphTest extends AbstractSymtabTest {

    @Test
    public void test() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("de.monticore.lang.monticar.semantics.loops.Test01", EMAComponentSymbol.KIND).orElse(null);
        EMAGraphTransformation transformation = new EMAGraphTransformation();
        EMAGraph graph = transformation.transform(component);
    }
}
