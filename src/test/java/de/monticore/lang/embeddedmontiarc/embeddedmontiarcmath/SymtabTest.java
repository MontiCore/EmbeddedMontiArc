/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.helper.TypeHelper;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SymtabTest extends AbstractSymtabTest {

  @Test
  public void testParsing() throws Exception {
    EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
    assertTrue(parser.parse("src/test/resources/adapterTest/A.emam").isPresent());
  }
  @Ignore
  @Test
  public void testSimilarityImageMatrixAdapter(){
    Scope symTab = createSymTab("src/test/resources");
    ComponentSymbol a = symTab.<ComponentSymbol>resolve("detection.SimilarityImageMatrixAdapter", ComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }
  @Ignore
  @Test
  public void testSimilarityImageMatrixCalculator(){
    Scope symTab = createSymTab("src/test/resources");
    ComponentSymbol a = symTab.<ComponentSymbol>resolve("detection.SimilarityImageMatrixCalculator", ComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }
  @Ignore
  @Test
  public void testEigenSolver(){
    Scope symTab = createSymTab("src/test/resources");
    ComponentSymbol a = symTab.<ComponentSymbol>resolve("detection.EigenSolver", ComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }
@Ignore
  @Test
  public void testKMeansClustering(){
    Scope symTab = createSymTab("src/test/resources");
    ComponentSymbol a = symTab.<ComponentSymbol>resolve("detection.KMeansClustering", ComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }

  @Ignore
  @Test
  public void testDetection(){
    Scope symTab = createSymTab("src/test/resources");
    ComponentSymbol a = symTab.<ComponentSymbol>resolve("detection.ObjectDetector", ComponentSymbol.KIND).orElse(null);
    assertNotNull(a);

  }

  @Ignore
  @Test
  public void testLookUp(){

    Scope symTab = createSymTab("src/test/resources");
    ComponentSymbol a1 = symTab.<ComponentSymbol>resolve("fas.basicLibrary.LookUp", ComponentSymbol.KIND).orElse(null);
    assertNotNull(a1);

    ComponentSymbol a2 = symTab.<ComponentSymbol>resolve("test.LookUpTest", ComponentSymbol.KIND).orElse(null);
    assertNotNull(a2);

    ComponentInstanceSymbol a3 = symTab.<ComponentInstanceSymbol>resolve("test.LookUpTest.look1",ComponentInstanceSymbol.KIND).orElse(null);
    assertNotNull(a3);
    Log.debug(a3.toString(),"info");

    TypeHelper.getUnitNumberFromUnitNumberTypeArgument((ASTSubComponent) a3.getAstNode().get(),0);
    }

    /*
  @Test
  public void testSymbolTableCreatorDelegation1() {
    Scope symTab = createSymTab("src/test/resources");
    ComponentSymbol a = symTab.<ComponentSymbol>resolve("adapterTest.A", ComponentSymbol.KIND).orElse(null);
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
     // System.out.println(statement.getSymbol().toString());
    }
  }


  @Test
  public void testAddition(){
    Scope symTab = createSymTab("src/test/resources");

  }
  */
}
