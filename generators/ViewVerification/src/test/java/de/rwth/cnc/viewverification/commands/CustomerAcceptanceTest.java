/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.util.Collection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewSymbol;
import de.rwth.cnc.LogConfig;
import de.rwth.cnc.viewverification.EmbeddedMontiArcLoader;
import de.rwth.cnc.viewverification.EmbeddedMontiViewLoader;
import org.apache.commons.io.FileUtils;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

public class CustomerAcceptanceTest {

  final String TESTDIR = "src/test/resources/evalInput_edited/";
  final String TESTDIRCA = "src/test/resources/";

  @BeforeClass
  public static void init() {
    LogConfig.init();
  }

  @Test
  public void checkPositiveEffectorPumpingStation() {
    TestHelper.checkModelViewWitnessResult(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PositiveEffector", TESTDIR, "pumpStationExample.Witness_PositiveEffector");
  }

  @Test
  public void checkLoopPositiveEffectorCustomerAcceptance() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.LoopComponent", TESTDIRCA, "customerAcceptanceTests.LoopPositiveEffector", TESTDIRCA, "customerAcceptanceTests.Witness_LoopPositiveEffector");
  }

  @Test
  public void checkNestedLoopPositiveEffectorCustomerAcceptance() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.NestedLoopComponent", TESTDIRCA, "customerAcceptanceTests.NestedLoopPositiveEffector", TESTDIRCA, "customerAcceptanceTests.Witness_NestedLoopPositiveEffector");
  }

  @Test
  public void checkNestedLoopPositiveEffector2CustomerAcceptance() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.NestedLoopComponent", TESTDIRCA, "customerAcceptanceTests.NestedLoopPositiveEffector2", TESTDIRCA, "customerAcceptanceTests.Witness_NestedLoopPositiveEffector2");
  }

  @Test
  public void checkNegativeEffectorPumpingStation() {
    TestHelper.checkModelViewWitnessResult(TESTDIR, "pumpStationExample.SensorReading", TESTDIR, "pumpStationExample.NegativeEffector", TESTDIR, "pumpStationExample.Witness_NegativeEffector");
  }

  //char / short problem!
  @Ignore
  @Test
  public void checkNegativeEffectorEditedPumpingStation() {
    TestHelper.checkModelViewWitnessResult(TESTDIR, "pumpStationExample.SensorReadingEdited", TESTDIR, "pumpStationExample.NegativeEffectorEdited", TESTDIR, "pumpStationExample.Witness_NegativeEffectorEdited");
  }

  @Ignore
  @Test
  public void checkCharShortDiff() {
    de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol cs = EmbeddedMontiArcLoader.loadComponentSymbol(TESTDIRCA, "customerAcceptanceTests.CharShortComponent");
//    System.out.println(cs.toString());
    Collection<PortSymbol> collection = cs.getPorts();
    for (PortSymbol ps : collection) {
      ps.getTypeReference().getName();
    }
    //bug makes char to short!
  }

  @Test
  public void checkHierarchicalPositive() {
    TestHelper.checkModelViewWitnessResult(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.HierarchicalPositive", TESTDIR, "pumpStationExample.Witness_HierarchicalPositive");
  }

  @Test
  public void checkHierarchicalNegative() {
    TestHelper.checkModelViewWitnessResult(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.HierarchicalNegative", TESTDIR, "pumpStationExample.Witness_HierarchicalNegative");
  }

  @Test
  public void checkConnTest1() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.ConnectorTestModel", TESTDIRCA, "customerAcceptanceTests.ConnectorTestView1", TESTDIRCA, "customerAcceptanceTests.Witness_ConnectorTestView1");
  }

  @Test
  public void checkConnTest2() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.ConnectorTestModel", TESTDIRCA, "customerAcceptanceTests.ConnectorTestView2", TESTDIRCA, "customerAcceptanceTests.Witness_ConnectorTestView2");
  }

  @Test
  public void checkConnTest3() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.ConnectorTestModel", TESTDIRCA, "customerAcceptanceTests.ConnectorTestView3", TESTDIRCA, "customerAcceptanceTests.Witness_ConnectorTestView3");
  }

  @Test
  public void checkConnTest4() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.ConnectorTestModel", TESTDIRCA, "customerAcceptanceTests.ConnectorTestView4", TESTDIRCA, "customerAcceptanceTests.Witness_ConnectorTestView4");
  }

  @Test
  public void checkUnitsPositive() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.UnitModel", TESTDIRCA, "customerAcceptanceTests.UnitModelViewPositive", TESTDIRCA, "customerAcceptanceTests.Witness_UnitModelViewPositive");
  }

  @Test
  public void checkUnitsNegative_type() throws IOException {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.UnitModel", TESTDIRCA, "customerAcceptanceTests.UnitModelViewNegative1", TESTDIRCA, "customerAcceptanceTests.Witness_UnitModelViewNegative1");
    String content = FileUtils.readFileToString(new File("target\\generated-witnesses\\negative\\UnitModelViewNegative1\\customerAcceptanceTests\\InterfaceMismatch0.emv".replace('\\', '/')));
    assertTrue(content.length() > 20);
    assertFalse(content.contains("SIUnitRangesType"));
  }

  @Test
  public void checkUnitsNegative_range() {
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.UnitModel", TESTDIRCA, "customerAcceptanceTests.UnitModelViewNegative2", TESTDIRCA, "customerAcceptanceTests.Witness_UnitModelViewNegative2");
  }
}
