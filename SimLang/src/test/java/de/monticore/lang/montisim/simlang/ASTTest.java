package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang._ast.*;
import de.monticore.lang.montisim.simlang._visitor.SimLangVisitor;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

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
    node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("60ms");
  }
  public void visit(ASTSimulationLoopFrequency node) {
    assert node.getAlternativeInput().getRange().get().getStart().get().getUnitNumber().get().getTUnitNumber().get().equals("60m");
    assert node.getAlternativeInput().getRange().get().getStep().get().getUnitNumber().get().getTUnitNumber().get().equals("1m");
    assert node.getAlternativeInput().getRange().get().getEnd().get().getUnitNumber().get().getTUnitNumber().get().equals("80m");
  }
  public void visit(ASTSimulationDuration node) {
    assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(0).getTUnitNumber().get().equals("50m");
    assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(1).getTUnitNumber().get().equals("4h") ;
    assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(2).getTUnitNumber().get().equals("5h") ;
    assert node.getAlternativeInput().getUnitNumberList().get().getUnitNumbers().get(3).getTUnitNumber().get().equals("6h") ;
  }
  public void visit(ASTTime node) {
    node.getSingleTime().get().getHours().getTUnitNumber().get().equals("1") ;
    assert node.getSingleTime().get().getMinutes().getTUnitNumber().get().equals("22") ;
    assert node.getSingleTime().get().getSeconds().get().getTUnitNumber().get().equals("33") ;
    assert node.getSingleTime().get().getMilliseconds().get().getTUnitNumber().get().equals("444");
  }
  public void visit(ASTMapPath node) {
    assert node.getMapPath().equals("Maps");
  }
  public void visit(ASTMapName node) {
    assert node.getMapName().equals("HorsterDreieck");
    assert node.getFileFormat().equals("osm");
  }
  public void visit(ASTMapHeight node) {
    //assert node.getHeightMode() == 0;
  }
  public void visit(ASTMapOverlap node) {
    assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("10");
  }
  public void visit(ASTMapSectorWidth node) {
    assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("100");
  }
  public void visit(ASTMapSectorHeight node) {
    assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("100");
  }
  public void visit(ASTTimeout node) {
    assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("12h");
  }
  public void visit(ASTGravity node) {
    assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("12m/s^2");
  }
  public void visit(ASTPedestrianDensity node) {
    assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("2.0");
  }
  public void visit(ASTMaxSectorUsers node) {
    assert node.getAlternativeInput().getUnitNumber().get().getTUnitNumber().get().equals("1234");
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
