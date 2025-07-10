/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import de.rwth.cnc.LogConfig;
import de.rwth.cnc.model.CnCView;
import de.rwth.cnc.model.Component;
import de.rwth.cnc.viewverification.EmbeddedMontiViewLoader;
import org.junit.BeforeClass;
import org.junit.Test;

public class EmbeddedMontiViewLoaderTest {

  private static final String TESTDIR = "src/test/resources/";

  @BeforeClass
  public static void init() {
    LogConfig.init();
  }

  @Test
  public void exampleConnectCompleteness() {
    CnCView view = EmbeddedMontiViewLoader.loadView(TESTDIR, "example.ExampleConnect");

    assert null != view;
    assert view.getComponents().size() == 2 : "components wrong " + view.getComponents().size();
    assert view.getConnections().size() == 1 : "connectors wrong";
    assert view.getEffectors().size() == 0 : "effectors wrong";
    assert view.getPorts().size() == 2 : "getPorts wrong";
    assert view.getTopLevelComponentNames().size() == 1 : "toplevelcomponents wrong";
    assert view.getPortTypes().size() == 1 : "porttypes wrong";
  }

  @Test
  public void exampleEffectCompleteness() {
    CnCView view = EmbeddedMontiViewLoader.loadView(TESTDIR, "example.ExampleEffect");

    assert null != view;
    assert view.getComponents().size() == 1 : "components wrong " + view.getComponents().size();
    assert view.getConnections().size() == 0 : "connectors wrong";
    assert view.getEffectors().size() == 1 : "effectors wrong";
    assert view.getPorts().size() == 2 : "getPorts wrong";
    assert view.getTopLevelComponentNames().size() == 1 : "toplevelcomponents wrong";
    assert view.getPortTypes().size() == 1 : "porttypes wrong";
  }

  @Test
  public void exampleHierarchyCompleteness() {
    CnCView view = EmbeddedMontiViewLoader.loadView(TESTDIR, "example.ExampleHierarchy");

    assert null != view;
    assert view.getComponents().size() == 3 : "components wrong " + view.getComponents().size();
    assert view.getConnections().size() == 3 : "connectors wrong";
    assert view.getEffectors().size() == 0 : "effectors wrong";
    assert view.getPorts().size() == 4 : "getPorts wrong";
    assert view.getTopLevelComponentNames().size() == 1 : "toplevelcomponents wrong";
    assert view.getPortTypes().size() == 1 : "porttypes wrong";
  }

  @Test
  public void exampleTextualAnonymousPortsCompleteness() {
    CnCView view = EmbeddedMontiViewLoader.loadView(TESTDIR, "example.ExampleTextualAnonymousPorts");

    assert null != view;
    assert view.getComponents().size() == 4 : "components wrong " + view.getComponents().size();
    assert view.getConnections().size() == 2 : "connectors wrong";
    assert view.getEffectors().size() == 0 : "effectors wrong";
    assert view.getPorts().size() == 4 : "getPorts wrong";
    assert view.getTopLevelComponentNames().size() == 1 : "toplevelcomponents wrong";
    assert view.getPortTypes().size() == 1 : "porttypes wrong";
  }

  @Test
  public void FlightSystemTest() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/evalInput_edited/", "avionicsSystemExample.ConnectPilotDisplayAndPCM");

    assert null != view;
    assert view.getComponents().size() == 3 : "components wrong " + view.getComponents().size();
    assert view.getConnections().size() == 1 : "connectors wrong";
    assert view.getEffectors().size() == 0 : "effectors wrong";
    assert view.getPorts().size() == 0 : "getPorts wrong";
    assert view.getTopLevelComponentNames().size() == 1 : "toplevelcomponents wrong";
    assert view.getPortTypes().size() == 0 : "porttypes wrong";
  }

  @Test
  public void FlightSystemTest_Names() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/evalInput_edited/", "avionicsSystemExample.ConnectPilotDisplayAndPCM");

    assert null != view;

    for (Component c : view.getComponents()) {
      if (view.getTopLevelComponentNames().contains(c.getName()))
        assert Character.isUpperCase(c.getName().toCharArray()[0]);
      else
        assert Character.isLowerCase(c.getName().toCharArray()[0]);
    }
  }

  @Test
  public void loadSimpleTest00() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test00");
    assert null != view;
  }

  @Test
  public void loadSimpleTest01() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test01");
    assert null != view;
  }

  @Test
  public void loadSimpleTest02() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test02");
    assert null != view;
  }

  @Test
  public void loadSimpleTest03() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test03");
    assert null != view;
  }

  @Test
  public void loadSimpleTest04() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test04");
    assert null != view;
  }

  @Test
  public void loadSimpleTest05() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test05");
    assert null != view;
  }

  @Test
  public void loadSimpleTest06() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test06");
    assert null != view;
  }

  @Test
  public void loadSimpleTest07() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test07");
    assert null != view;
  }

  @Test
  public void loadSimpleTest08() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test08");
    assert null != view;
  }

  @Test
  public void loadSimpleTest09() {
    CnCView view = EmbeddedMontiViewLoader.loadView("src/test/resources/", "simpleTests.Test09");
    assert null != view;
  }

}
