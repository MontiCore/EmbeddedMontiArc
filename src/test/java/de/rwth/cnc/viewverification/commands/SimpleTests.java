/**
 * ******************************************************************************
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
package de.rwth.cnc.viewverification.commands;

import java.util.List;

import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.Test;

public class SimpleTests {

  private static final String TESTDIR = "src/test/resources/";

  @Test
  public void simpleTest00() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test00");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest01() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test01");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest02() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test02");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest03() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test03");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest04() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test04");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest05() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test05");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest06() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test06");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest07() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test07");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest08() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test08");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest09() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test09");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert inconsistencies.isEmpty() : "There should not be inconsistencies!";
    System.out.println("Test successful");
  }

  @Test
  public void simpleTest10() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test10");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest11() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.AC", TESTDIR, "simpleTests.Test11");

    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_IFC_PosTest() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ifcompleteness.BComp", TESTDIR,
            "simpleTests.ifcompleteness.PosTest");

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }


  @Test
  public void simpleTest_IFC_NegTest1() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ifcompleteness.BComp", TESTDIR,
            "simpleTests.ifcompleteness.NegTest1");

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_IFC_NegTest2() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ifcompleteness.BComp", TESTDIR,
            "simpleTests.ifcompleteness.NegTest2");

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_atomic_PosTest() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.atomic.CComp", TESTDIR,
            "simpleTests.atomic.PosTest");

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_atomic_NegTest() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.atomic.CComp", TESTDIR,
            "simpleTests.atomic.NegTest");

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_atomic_PosTest_ignoreCompName() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.atomic.CComp", TESTDIR,
            "simpleTests.atomic.PosTest", false, true);

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_ignoreCompName_PosViewA() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ignoreCompNames.Car", TESTDIR,
            "simpleTests.ignoreCompNames.PosViewA", false, true);

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert (inconsistencies.isEmpty()) : "There shouldnt be inconsistencies!";

    System.out.println("Test successful");
  }

  @Test
  public void simpleTest_ignoreCompName_NegViewA() {
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "simpleTests.ignoreCompNames.Car", TESTDIR,
            "simpleTests.ignoreCompNames.NegViewA", false, true);

    System.out.println("##############");
    inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
    assert !(inconsistencies.isEmpty()) : "There >>should<< be inconsistencies!";

    System.out.println("Test successful");
  }


}