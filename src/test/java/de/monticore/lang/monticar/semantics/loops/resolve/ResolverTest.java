package de.monticore.lang.monticar.semantics.loops.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import org.junit.Test;

public class ResolverTest {

    @Test
    public void createSymTab() {
        LogStub.init();
        Log.enableFailQuick(true);
        String model = "de.monticore.lang.monticar.semantics.loops.simpleLoop";
        GlobalScope symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources");
        Resolver resolver = new Resolver(model);
        TaggingResolver symTab1 = resolver.createSymTab(symTab);
        EMAComponentInstanceSymbol component = symTab1.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
        assert(component != null);
        component.toString();
    }
}