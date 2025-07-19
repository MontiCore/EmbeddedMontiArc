package de.monticore.lang.monticar.semantics.loops;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.EMAGraphTransformation;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.symboltable.Scope;
import org.junit.Ignore;
import org.junit.Test;

public class TransformGraphTest extends AbstractSymtabTest {

    @Ignore
    @Test
    public void test() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol component = symTab.<EMAComponentInstanceSymbol>resolve("de.monticore.lang.monticar.semantics.loops.test01", EMAComponentInstanceSymbol.KIND).orElse(null);
        EMAGraphTransformation transformation = new EMAGraphTransformation();
        EMAGraph graph = transformation.transform(component);
    }
}
