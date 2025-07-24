/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang._ast.*;
import de.monticore.lang.montisim.simlang._visitor.SimLangVisitor;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;
import org.junit.Before;
import org.junit.Test;

import javax.measure.unit.Unit;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

public class ASTTest implements SimLangVisitor {
  private SimLangVisitor realThis = this;

  int numEVehicles = 0;
  int numPVehicles = 0;
  int numRVehicles = 0;
  int numPed = 0;
  int numChannels = 0;

  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(false);
  }

  @Test
  public void testAST() throws Exception {
    test("src/test/resources/test/ast/FullExample.sim");
    if (Log.getErrorCount() > 0) {
      throw new Exception("Test Failed, found errors");
    }
    Log.info("Completed: AST Test.","");
  }

  private void test(String file) {
    final ASTSimLangCompilationUnit ast = SimLangTool.parse(file);

    ast.accept(realThis);

    assert numPed == 1;
    assert numEVehicles == 1;
    assert numPVehicles == 1;
    assert numRVehicles == 2;
    assert numChannels == 2;
  }

  public void visit(ASTSimulation node) {
    assert node.getName().equals("FullExample");
  }
  public void visit(ASTSimulationRenderFrequency node) {
    // node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("60ms");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(60d);
    assert node.getAlternativeInput().getNumberWithUnit().getUnit().equals(Unit.valueOf("ms"));
  }
  public void visit(ASTSimulationLoopFrequency node) {
    // assert node.getAlternativeInput().getRange().get().getStart().get().getUnitNumber().get().getTUnitNumber().get().equals("60m");
    assert node.getAlternativeInput().getRange().getStartValue().equals(Rational.valueOf("60"));
    assert node.getAlternativeInput().getRange().getStartUnit().equals(Unit.valueOf("m"));
    // assert node.getAlternativeInput().getRange().get().getStep().get().getUnitNumber().get().getTUnitNumber().get().equals("1m");
    assert node.getAlternativeInput().getRange().getStepValue().equals(Rational.valueOf("1"));
    assert node.getAlternativeInput().getRange().getStepUnit().equals(Unit.valueOf("m"));
    // assert node.getAlternativeInput().getRange().get().getEnd().get().getUnitNumber().get().getTUnitNumber().get().equals("80m");
    assert node.getAlternativeInput().getRange().getEndValue().equals(Rational.valueOf("80"));
    assert node.getAlternativeInput().getRange().getEndUnit().equals(Unit.valueOf("m"));
  }
  public void visit(ASTSimulationDuration node) {
    List<ASTNumberWithUnit> x = node.getAlternativeInput().getNumberWithUnitList().getNumberWithUnitList();
    // assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(0).getTUnitNumber().get().equals("50m");
    assert x.get(0).getNumber().get().equals(50d);
    assert x.get(0).getUnit().equals(Unit.valueOf("m"));
    // assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(1).getTUnitNumber().get().equals("4h") ;
    assert x.get(1).getNumber().get().equals(4d);
    assert x.get(1).getUnit().equals(Unit.valueOf("h"));
    // assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(2).getTUnitNumber().get().equals("5h") ;
    assert x.get(2).getNumber().get().equals(5d);
    assert x.get(2).getUnit().equals(Unit.valueOf("h"));
    // assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(3).getTUnitNumber().get().equals("6h") ;
    assert x.get(3).getNumber().get().equals(6d);
    assert x.get(3).getUnit().equals(Unit.valueOf("h"));
  }
  public void visit(ASTTime node) {
    // node.getSingleTime().get().getHours().getTUnitNumber().get().equals("1") ;
    assert node.getSingleTime().getHours().getNumber().get().equals(1d);
    // assert node.getSingleTime().get().getMinutes().getTUnitNumber().get().equals("22") ;
    assert node.getSingleTime().getMinutes().getNumber().get().equals(22d);
    // assert node.getSingleTime().get().getSeconds().get().getTUnitNumber().get().equals("33") ;
    assert node.getSingleTime().getSeconds().getNumber().get().equals(33d);
    // assert node.getSingleTime().get().getMilliseconds().get().getTUnitNumber().get().equals("444");
    assert node.getSingleTime().getMilliseconds().getNumber().get().equals(444d);
  }
  public void visit(ASTMapPath node) {
    assert node.getMapPath().equals("Maps");
  }
  public void visit(ASTMapName node) {
    assert node.getMapName().equals("HorsterDreieck");
    assert node.getFileFormat().equals("osm");
  }
  public void visit(ASTMapHeight node) {
    // flat = 3; random = 6; asdf = 0
    // assert node.getHeightMode() == 0;
    assert node.getHeightMode() == ASTConstantsSimLang.FLAT;
  }
  public void visit(ASTMapOverlap node) {
    // assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("10");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(10d);
  }
  public void visit(ASTMapSectorWidth node) {
    // assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("100");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(100d);
  }
  public void visit(ASTMapSectorHeight node) {
    // assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("100");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(100d);
  }
  public void visit(ASTTimeout node) {
    // assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("12h");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(12d);
    assert node.getAlternativeInput().getNumberWithUnit().getUnit().equals(Unit.valueOf("h"));
  }
  public void visit(ASTGravity node) {
    // assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("12m/s^2");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(12d);
    assert node.getAlternativeInput().getNumberWithUnit().getUnit().equals(Unit.valueOf("m/s^2"));
  }
  public void visit(ASTPedestrianDensity node) {
    // assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("2.0");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(2.0);
  }
  public void visit(ASTMaxSectorUsers node) {
    // assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("1234");
    assert node.getAlternativeInput().getNumberWithUnit().getNumber().get().equals(1234d);
  }
  public void visit(ASTPedestrians node) {
    this.numPed++;
  }
  public void visit(ASTExplicitVehicle node) {
    this.numEVehicles++;
  }
  public void visit(ASTPathedVehicle node) {
    this.numPVehicles++;
  }
  public void visit(ASTRandomVehicle node) {
    this.numRVehicles++;
  }
  public void visit(ASTChannel node) {
    this.numChannels++;
  }
}
