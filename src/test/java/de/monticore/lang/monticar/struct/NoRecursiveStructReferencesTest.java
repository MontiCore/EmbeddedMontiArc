/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct;

import de.monticore.lang.monticar.struct._ast.ASTStructNode;
import de.monticore.lang.monticar.struct._cocos.StructCoCoChecker;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.struct.coco.NoRecursiveStructReferences;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class NoRecursiveStructReferencesTest {

    private Scope symTab;
    private StructCoCoChecker checker;

    @Before
    public void setup() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
        symTab = Utils.createSymTab("src/test/resources");
        checker = new StructCoCoChecker()
                .addCoCo(new NoRecursiveStructReferences());
    }

    @Test
    public void testValidStructs() {
        String[] validStructs = new String[]{
                "test.symtable.MyFancyStruct1",
                "test.symtable.sub1.S1",
                "test.symtable.sub2.T1",
                "test.symtable.sub3.K1",
                "test.coco.valid.J1",
                "test.coco.valid.K1",
                "test.coco.valid.S1",
                "test.coco.valid.StructWithVeryVeryVeryLongName",
                "test.coco.valid.T1",
                "test.coco.valid.W1",
        };
        for (String structFullName : validStructs) {
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            ASTStructNode ast = (ASTStructNode) struct.getAstNode().orElse(null);
            Assert.assertNotNull(ast);
            Log.getFindings().clear();
            checker.checkAll(ast);
            Assert.assertTrue(Log.getFindings().isEmpty());
        }
    }

    @Test
    public void testSelfReferences() {
        for (int i = 1; i <= 4; i++) {
            String structFullName = String.format("test.symtable.ErrSelfReference%s", i);
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            ASTStructNode ast = (ASTStructNode) struct.getAstNode().orElse(null);
            Assert.assertNotNull(ast);
            Log.getFindings().clear();
            checker.checkAll(ast);
            Assert.assertFalse(Log.getFindings().isEmpty());
        }
    }

    @Test
    public void testCycles() {
        String[] structsWithCycles = new String[]{
                "test.symtable.sub1.ErrCycle1",
                "test.symtable.sub2.ErrCycle1",
                "test.symtable.sub3.ErrCycle1",
                "test.symtable.sub1.ErrCycle2",
                "test.symtable.sub2.ErrCycle2",
                "test.symtable.sub3.ErrCycle2"
        };
        for (String structFullName : structsWithCycles) {
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            ASTStructNode ast = (ASTStructNode) struct.getAstNode().orElse(null);
            Assert.assertNotNull(ast);
            Log.getFindings().clear();
            checker.checkAll(ast);
            Assert.assertFalse(Log.getFindings().isEmpty());
        }
    }

    @Test
    public void testNonExistentReferences() {
        String[] structsWithNonExistentReferences = new String[]{
                "test.symtable.ErrNonExistentReferences"
        };
        for (String structFullName : structsWithNonExistentReferences) {
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            ASTStructNode ast = (ASTStructNode) struct.getAstNode().orElse(null);
            Assert.assertNotNull(ast);
            Log.getFindings().clear();
            checker.checkAll(ast);
            Assert.assertFalse(Log.getFindings().isEmpty());
        }
    }
}
