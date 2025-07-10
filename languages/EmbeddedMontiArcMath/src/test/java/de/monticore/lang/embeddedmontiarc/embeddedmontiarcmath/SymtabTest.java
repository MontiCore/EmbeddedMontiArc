/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.helper.TypeHelper;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbolKind;
import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

import java.util.Collection;
import java.util.stream.Collectors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SymtabTest extends AbstractSymtabTest {

  @Test
  public void testParsing() throws Exception {
    EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
    assertTrue(parser.parse("src/test/resources/emam/adapterTest/A.emam").isPresent());
  }
  @Ignore
  @Test
  public void testSimilarityImageMatrixAdapter(){
    Scope symTab = createSymTab("src/test/resources/emam");
    EMAComponentSymbol a = symTab.<EMAComponentSymbol>resolve("detection.SimilarityImageMatrixAdapter", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }
  @Ignore
  @Test
  public void testSimilarityImageMatrixCalculator(){
    Scope symTab = createSymTab("src/test/resources/emam");
    EMAComponentSymbol a = symTab.<EMAComponentSymbol>resolve("detection.SimilarityImageMatrixCalculator", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }
  @Ignore
  @Test
  public void testEigenSolver(){
    Scope symTab = createSymTab("src/test/resources/emam");
    EMAComponentSymbol a = symTab.<EMAComponentSymbol>resolve("detection.EigenSolver", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }
@Ignore
  @Test
  public void testKMeansClustering(){
    Scope symTab = createSymTab("src/test/resources/emam");
    EMAComponentSymbol a = symTab.<EMAComponentSymbol>resolve("detection.KMeansClustering", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }

  @Ignore
  @Test
  public void testDetection(){
    Scope symTab = createSymTab("src/test/resources/emam");
    EMAComponentSymbol a = symTab.<EMAComponentSymbol>resolve("detection.ObjectDetector", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }

  @Ignore
  @Test
  public void testLookUp(){

    Scope symTab = createSymTab("src/test/resources/emam");
    EMAComponentSymbol a1 = symTab.<EMAComponentSymbol>resolve("fas.basicLibrary.LookUp", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a1);

    EMAComponentSymbol a2 = symTab.<EMAComponentSymbol>resolve("test.LookUpTest", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a2);

    EMAComponentInstanceSymbol a3 = symTab.<EMAComponentInstanceSymbol>resolve("test.LookUpTest.look1",EMAComponentInstanceSymbol.KIND).orElse(null);
    assertNotNull(a3);
    Log.debug(a3.toString(),"info");

    TypeHelper.getUnitNumberFromUnitNumberTypeArgument((ASTSubComponent) a3.getAstNode().get(),0);
    }

    @Test
    public void testParameterInstanceInstance() {
        Scope symTab = createSymTab("src/test/resources/emam");
        EMAComponentInstanceSymbol a =
                symTab.<EMAComponentInstanceSymbol>resolve("test.simpleParameterInstanceInstance",
                        EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(a);
        EMAComponentInstanceSymbol sub1 = a.getSubComponent("simpleParameterInstance1").orElse(null);
        EMAComponentInstanceSymbol sub2 = a.getSubComponent("simpleParameterInstance2").orElse(null);
        assertNotNull(sub1);
        assertNotNull(sub2);
        assertEquals("2", ((MathExpressionSymbol) sub1.getArguments().get(0).getSymbol()).getTextualRepresentation());
        assertEquals("9", ((MathExpressionSymbol) sub2.getArguments().get(0).getSymbol()).getTextualRepresentation());
        EMAComponentInstanceSymbol sub11 = sub1.getSubComponent("simpleParameter").orElse(null);
        EMAComponentInstanceSymbol sub21 = sub2.getSubComponent("simpleParameter").orElse(null);
        assertNotNull(sub11);
        assertNotNull(sub21);
        assertEquals("2", ((MathExpressionSymbol) sub11.getArguments().get(0).getSymbol()).getTextualRepresentation());
        assertEquals("9", ((MathExpressionSymbol) sub21.getArguments().get(0).getSymbol()).getTextualRepresentation());
        Collection<MathExpressionSymbol> symbols11 = sub11.getSpannedScope().resolveLocally(MathExpressionSymbol.KIND);
        Collection<MathExpressionSymbol> symbols21 = sub21.getSpannedScope().resolveLocally(MathExpressionSymbol.KIND);
        symbols11 =
                symbols11.stream().filter(s -> s instanceof MathArithmeticExpressionSymbol)
                        .collect(Collectors.toList());
        symbols21 =
                symbols21.stream().filter(s -> s instanceof MathArithmeticExpressionSymbol)
                        .collect(Collectors.toList());
        assert (symbols11.size() == 1);
        assert (symbols21.size() == 1);
        MathArithmeticExpressionSymbol assignment1 = (MathArithmeticExpressionSymbol) symbols11.iterator().next();
        MathArithmeticExpressionSymbol assignment2 = (MathArithmeticExpressionSymbol) symbols21.iterator().next();
        assertEquals("2", assignment1.getRightExpression().getTextualRepresentation());
        assertEquals("9", assignment2.getRightExpression().getTextualRepresentation());
    }

    @Test
    public void testCubeValid() {
        Scope symTab = createSymTab("src/test/resources/emam/cube/main", "src/test/resources/emam/cube/test");
        EMAComponentInstanceSymbol a =
                symTab.<EMAComponentInstanceSymbol>resolve("testB.cube_TestWrapper",
                        EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(a);
    }

    @Test
    public void testBoolValid() {
        Scope symTab = createSymTab("src/test/resources/emam/valid/main", "src/test/resources/emam/valid/test");
        EMAComponentInstanceSymbol a =
                symTab.<EMAComponentInstanceSymbol>resolve("testA.and_TestWrapper",
                        EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(a);
    }

    /*
  @Test
  public void testSymbolTableCreatorDelegation1() {
    Scope symTab = createSymTab("src/test/resources");
    EMAComponentSymbol a = symTab.<EMAComponentSymbol>resolve("adapterTest.A", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(a);
  }

  @Test
  public void testSymbolTableCreatorDelegation2() {
    Scope symTab = createSymTab("src/test/resources");
    PortSymbol p1 = symTab.<PortSymbol>resolve("adapterTest.A.p1[1]", PortSymbol.KIND).orElse(null);
    assertNotNull(p1);
  }

  @Test
  public void testSymbolTableCreatorDelegation3() {
    Scope symTab = createSymTab("src/test/resources");
    PortArraySymbol p1 = symTab.<PortArraySymbol>resolve("adapterTest.A.p1", PortArraySymbol.KIND).orElse(null);
    assertNotNull(p1);
  }

  @Test
  public void testSymbolTableCreatorDelegation4() {
    Scope symTab = createSymTab("src/test/resources");
    MathVariableDeclarationSymbol j = symTab.<MathVariableDeclarationSymbol>resolve(
            "adapterTest.A.j", MathVariableDeclarationSymbol.KIND).orElse(null);
    assertNotNull(j);
  }
  @Ignore
  @Test
  public void testAdaption() {
    Scope symTab = createSymTab("src/test/resources");
    Log.debug(symTab.toString(),"Scope:");

    // A.p1 is actually a port, let's see whether we will also find it as a MathVariableDeclaration
    MathVariableDeclarationSymbol p1 = symTab.<MathVariableDeclarationSymbol>resolve("adapterTest.A.p1", MathVariableDeclarationSymbol.KIND).orElse(null);
    assertNotNull(p1);

    // port in  (0m : 100m) p1[9]
    assertTrue(p1.getRange().getStart().isPresent());
    assertEquals(ASTUnitNumber.valueOf(Rational.ZERO, Unit.valueOf("m")),
            p1.getRange().getStart().get());
    assertTrue(p1.getRange().getEnd().isPresent());
    assertEquals(ASTUnitNumber.valueOf(Rational.valueOf(100, 1), Unit.valueOf("m")),
            p1.getRange().getEnd().get());
    assertEquals(Arrays.asList(1, 9), p1.getDimensions()); // converted to a row vector

    //Mathstatements are available in symbol table now and provide information about order and type by
    //adding the relevant parts of the ast
    //System.out.println("Start!!!");
    MathStatementsSymbol symbol=symTab.<MathStatementsSymbol>resolve("adapterTest.A.MathStatements", MathStatementsSymbol.KIND).orElse(null);
    assertNotNull(symbol);
    for(ASTMathStatement statement:symbol.getAstMathStatements().getMathStatements()){
     // System.out.println(statement.getSymbolOpt().toString());
    }
  }


  @Test
  public void testAddition(){
    Scope symTab = createSymTab("src/test/resources");

  }
  */

  @Test
  public void testMathStatementsSymbol() {
    Scope symTab = createSymTab("src/test/resources/emam/");
    EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("test.Add", EMAComponentSymbol.KIND).orElse(null);
    MathStatementsSymbol statements = (MathStatementsSymbol) component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND).orElse(null);
    assertNotNull(statements);
    assertEquals(1, statements.getMathExpressionSymbols().size());
    assertEquals(MathAssignmentExpressionSymbol.class,  statements.getMathExpressionSymbols().get(0).getClass());
  }
}
