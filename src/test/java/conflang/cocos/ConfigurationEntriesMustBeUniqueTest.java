/* (c) https://github.com/MontiCore/monticore */
package conflang.cocos;

import conflang.AbstractTest;
import conflang.ConfLangMill;
import conflang._ast.ASTConfLang;
import conflang._cocos.ConfLangCoCoChecker;
import conflang._symboltable.ConfLangScopesGenitor;
import conflang._symboltable.IConfLangGlobalScope;
import conflang._visitor.ConfLangTraverser;
import de.monticore.io.paths.ModelPath;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Before;
import org.junit.BeforeClass;

import java.nio.file.Paths;

import static org.junit.Assert.assertEquals;

public class ConfigurationEntriesMustBeUniqueTest extends AbstractTest {

    private IConfLangGlobalScope globalScope;

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() throws RecognitionException {
        LogStub.init();
        Log.getFindings().clear();
        globalScope = ConfLangMill.globalScope();
        globalScope.clear();
    }

    public void testValid() {
        globalScope.setModelPath(new ModelPath(Paths.get("src/test/resources/conflang/cocos/valid")));

        ASTConfLang ast = parseModel("src/test/resources/conflang/cocos/valid/UglyConfiguration.conf");
        ConfLangScopesGenitor genitor = ConfLangMill.scopesGenitor();
        ConfLangTraverser traverser = ConfLangMill.traverser();
        traverser.setConfLangHandler(genitor);
        traverser.add4ConfLang(genitor);
        genitor.putOnStack(globalScope);
        genitor.createFromAST(ast);

        ConfLangCoCoChecker checker = new ConfLangCoCos().getCheckerForAllCoCos();
        checker.checkAll(ast);

        //The error log would be empty if configuration A was created from AST before being resolved
        assertEquals(0, Log.getErrorCount());
    }

//    @Test
//    public void testNotExistingTransitionSource() {
//        globalScope.setModelPath(new ModelPath(Paths.get("src/test/resources/conflang/cocos/invalid")));
//        ASTConfLang ast = parseModel("src/test/resources/conflang/cocos/invalid/NotExistingTransitionSource.aut");
//        ConfLangScopesGenitor genitor = ConfLangMill.scopesGenitor();
//        ConfLangTraverser traverser = ConfLangMill.traverser();
//        traverser.setConfLangHandler(genitor);
//        traverser.add4ConfLang(genitor);
//        genitor.putOnStack(globalScope);
//        genitor.createFromAST(ast);
//
//        ASTTransition transition = ast.get().get(0);
//
//        TransitionStatesExist coco = new TransitionStatesExist();
//        coco.check(transition);
//
//        Collection<Finding> expectedErrors = Collections.singletonList(
//                Finding.error("0xB4003 The source state of the transition does not exist.",
//                        new SourcePosition(6, 2)));
//
//        Assert.assertErrors(expectedErrors, Log.getFindings());
//    }

}
