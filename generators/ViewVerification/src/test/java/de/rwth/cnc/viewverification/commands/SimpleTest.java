/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import java.util.List;

import de.rwth.cnc.LogConfig;
import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.BeforeClass;
import org.junit.Test;

public class SimpleTest {

  private static final String TESTDIR = "src/test/resources/";

  @BeforeClass
  public static void init(){
    LogConfig.init();
  }

  @Test
  public void simpleTest00() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test00");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest01() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test01");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest02() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test02");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest03() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test03");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest04() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test04");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest05() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test05");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest06() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test06");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest07() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test07");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest08() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test08");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest09() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test09");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest10() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test10");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest11() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test11");

//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_IFC_PosTest() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ifcompleteness.BComp", TESTDIR,
            "simpleTests.ifcompleteness.PosTest");

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }


  @Test
  public void simpleTest_IFC_NegTest1() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ifcompleteness.BComp", TESTDIR,
            "simpleTests.ifcompleteness.NegTest1");

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_IFC_NegTest2() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ifcompleteness.BComp", TESTDIR,
            "simpleTests.ifcompleteness.NegTest2");

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_atomic_PosTest() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.atomic.CComp", TESTDIR,
            "simpleTests.atomic.PosTest");

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_atomic_NegTest() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.atomic.CComp", TESTDIR,
            "simpleTests.atomic.NegTest");

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_atomic_PosTest_ignoreCompName() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.atomic.CComp", TESTDIR,
            "simpleTests.atomic.PosTest", false, true);

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_ignoreCompName_PosViewA() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ignoreCompNames.Car", TESTDIR,
            "simpleTests.ignoreCompNames.PosViewA", false, true);

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_ignoreCompName_NegViewA() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ignoreCompNames.Car", TESTDIR,
        "simpleTests.ignoreCompNames.NegViewA", false, true);

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_unitmodel() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.UnitModel", TESTDIR,
        "simpleTests.UnitModelView", true, false);

//    System.out.println("##############");
//    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }


}
