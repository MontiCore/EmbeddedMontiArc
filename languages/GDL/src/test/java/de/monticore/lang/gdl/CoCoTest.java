package de.monticore.lang.gdl;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._symboltable.GDLScopesGenitor;
import de.monticore.lang.gdl._symboltable.IGDLGlobalScope;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.monticore.lang.gdl.cocos.AllCoCosChecker;
import de.se_rwth.commons.logging.Log;

public class CoCoTest {
    
    @Test
    public void testCoCosDoppelkopf() {
        ASTGame ast = GDLInterpreter.parse("src/test/resources/gdl/Doppelkopf.gdl");

        final IGDLGlobalScope gs = GDLMill.globalScope();
        gs.clear();

        final GDLScopesGenitor genitor = GDLMill.scopesGenitor();
        final GDLTraverser traverser = GDLMill.traverser();
        traverser.setGDLHandler(genitor);
        traverser.add4GDL(genitor);
        genitor.putOnStack(gs);

        gs.addSubScope(genitor.createFromAST(ast));

        final AllCoCosChecker checker = new AllCoCosChecker();
        checker.checkAll(ast);

        assertTrue("Doppelkopf contains unexpected CoCo Errors", Log.getErrorCount() == 0);
    }

}
