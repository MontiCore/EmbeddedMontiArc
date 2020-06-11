package de.monticore.lang.monticar.semantics.loops.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import static org.junit.Assert.*;

public class ResolverTest {

    @Test
    public void createSymTab() {
        GlobalScope symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources");
        Resolver resolver = new Resolver("de.monticore.lang.monticar.semantics.loops.SimpleLoop");
        TaggingResolver symTab1 = resolver.createSymTab(symTab);
        EMAComponentSymbol component = symTab1.<EMAComponentSymbol>resolve("de.monticore.lang.monticar.semantics.loops.SimpleLoop", EMAComponentSymbol.KIND).orElse(null);
        component.toString();
    }
}