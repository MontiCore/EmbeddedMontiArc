/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.helper.TypeHelper;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbolKind;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

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
