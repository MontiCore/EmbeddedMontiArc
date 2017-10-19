package de.rwth.cnc.viewverification.commands;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewSymbol;
import de.rwth.cnc.viewverification.EmbeddedMontiViewLoader;
import org.junit.Ignore;
import org.junit.Test;

public class CustomerAcceptanceTests {

  final String TESTDIR = "src/test/resources/evalInput_edited/";
  final String TESTDIRCA = "src/test/resources/";

  @Test
  public void checkPositiveEffectorPumpingStation() {
    TestHelper.checkModelViewWitnessResult(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PositiveEffector", TESTDIR, "pumpStationExample.Witness_PositiveEffector");
  }

  @Test
  public void checkAssertEquals() {
    final String folder1 = "src/test/resources/customerAcceptanceTests/assertEqualTest/witness1/";
    final String folder2 = "src/test/resources/customerAcceptanceTests/assertEqualTest/witness2/";
    ViewSymbol v1 = EmbeddedMontiViewLoader.loadViewSymbol(folder1, "pumpStationExample.AssertEqualsTest");
    ViewSymbol v2 = EmbeddedMontiViewLoader.loadViewSymbol(folder2, "pumpStationExample.AssertEqualsTest");
    TestHelper.assertEquality(v1, v2);
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
  public void checkNegativeEffectorPumpingStation() {
    TestHelper.checkModelViewWitnessResult(TESTDIR, "pumpStationExample.SensorReading", TESTDIR, "pumpStationExample.NegativeEffector", TESTDIR, "pumpStationExample.Witness_NegativeEffector");
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

  @Ignore
  @Test
  public void checkUnitsPositive() {
    // currently not possible as the type is always translated as "SIUnitRangesType"
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.UnitModel", TESTDIRCA, "customerAcceptanceTests.UnitModelViewPositive"
        ,TESTDIRCA, "customerAcceptanceTests.Witness_UnitModelViewPositive");

    // however, this currently works since both, the witness and the expected witness translate the type to SIUnitRangesType
    //therefore, remove the following assert when fixed!
    assert false;
  }

  @Ignore
  @Test
  public void checkUnitsNegative_type() {
    //currently not possible as the type is always translated as "SIUnitRangesType"
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.UnitModel", TESTDIRCA, "customerAcceptanceTests.UnitModelViewNegative1"
        ,TESTDIRCA, "customerAcceptanceTests.Witness_UnitModelViewNegative1");
  }

  @Ignore
  @Test
  public void checkUnitsNegative_range() {
    //currently not possible as the type is always translated as "SIUnitRangesType"
    TestHelper.checkModelViewWitnessResult(TESTDIRCA, "customerAcceptanceTests.UnitModel", TESTDIRCA, "customerAcceptanceTests.UnitModelViewNegative2"
        ,TESTDIRCA, "customerAcceptanceTests.Witness_UnitModelViewNegative2");
  }
}
