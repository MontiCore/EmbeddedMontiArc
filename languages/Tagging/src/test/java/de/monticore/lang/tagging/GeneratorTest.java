/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging;

import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedHashMap;
import java.util.Map;

import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.tagging.generator.TagSchemaGenerator;
import org.junit.Ignore;
import org.junit.Test;

/**
 * Created by Michael von Wenckstern on 08.06.2016.
 */
public class GeneratorTest {

  protected Path getPathFromRelativePath(String relPath) throws Exception {
    return Paths.get(URLClassLoader.newInstance(new URL[] { Paths.get(relPath).toUri().toURL() }).getURLs()[0].toURI());
  }

  @Test
  public void testSimpleTagType() throws Exception {
    GeneratorSetup setup = new GeneratorSetup();
    setup.setOutputDirectory(getPathFromRelativePath("src/test/resources").toFile());
        //getPathFromRelativePath("target/generated-sources/monticore/sourcecode").toFile());
    setup.setTracing(true);
    TagSchemaGenerator generator = new TagSchemaGenerator(setup);
    Map<String, String> symbolScopeMap = new LinkedHashMap<>();
    symbolScopeMap.put("Component", "NameScope");
    symbolScopeMap.put("Port", "NameScope");
    symbolScopeMap.put("ComponentInstance", "NameScope");
    symbolScopeMap.put("Connector", "ConnectorScope");
    generator.generate(Paths.get("TraceabilityTagSchema"), Paths.get("src/test/resources/nfp/"), symbolScopeMap);
  }

  @Test
  public void testValuedTagType() throws Exception {
    /*int x = 7;
    Number n = x;
    String s = n.toString();
    System.out.println(s);
    double d = 7;
    n = d;
    s = n.toString();
    System.out.println(s);
    System.out.println(s);
    boolean b = Unit.valueOf("mW").isCompatible(Power.UNIT);
    Unit.valueOf("mW").asType(Power.class);
    b = Unit.valueOf("s").isCompatible(Power.UNIT);
    Measure<? extends Number, Power> milliWatt = Measure.valueOf(7, (Unit<Power>)Unit.valueOf("mW"));
    System.out.println();*/
    GeneratorSetup setup = new GeneratorSetup();
    setup.setOutputDirectory(getPathFromRelativePath("src/test/resources/generator").toFile());
    setup.setTracing(true);
    TagSchemaGenerator generator = new TagSchemaGenerator(setup);
    Map<String, String> symbolScopeMap = new LinkedHashMap<>();
    symbolScopeMap.put("Component", "NameScope");
    symbolScopeMap.put("Port", "NameScope");
    symbolScopeMap.put("ComponentInstance", "NameScope");
    symbolScopeMap.put("Connector", "ConnectorScope");
    generator.generate(Paths.get("PowerConsumptionTagSchema"), Paths.get("src/test/resources/nfp/"), symbolScopeMap);
  }

  @Test
  public void testLatencyTag() throws Exception {
    GeneratorSetup setup = new GeneratorSetup();
    setup.setOutputDirectory(getPathFromRelativePath("src/test/resources/generator").toFile());
    setup.setTracing(true);
    TagSchemaGenerator generator = new TagSchemaGenerator(setup);
    Map<String, String> symbolScopeMap = new LinkedHashMap<>();
    symbolScopeMap.put("Component", "NameScope");
    symbolScopeMap.put("Port", "NameScope");
    symbolScopeMap.put("ComponentInstance", "NameScope");
    symbolScopeMap.put("Connector", "ConnectorScope");
    generator.generate(Paths.get("LatencyTagSchema"), Paths.get("src/test/resources/nfp/"), symbolScopeMap);
  }

  @Test
  public void testTransCostTag() throws Exception {
    GeneratorSetup setup = new GeneratorSetup();
    setup.setOutputDirectory(getPathFromRelativePath("src/test/resources/generator").toFile());
    setup.setTracing(true);
    TagSchemaGenerator generator = new TagSchemaGenerator(setup);
    Map<String, String> symbolScopeMap = new LinkedHashMap<>();
    symbolScopeMap.put("Component", "NameScope");
    symbolScopeMap.put("Port", "NameScope");
    symbolScopeMap.put("ComponentInstance", "NameScope");
    symbolScopeMap.put("Connector", "ConnectorScope");
    generator.generate(Paths.get("TransmissionCostsTagSchema"), Paths.get("src/test/resources/nfp/"), symbolScopeMap);
  }

  @Test
  public void testCompPowerTag() throws Exception {
    GeneratorSetup setup = new GeneratorSetup();
    setup.setOutputDirectory(getPathFromRelativePath("src/test/resources/generator").toFile());
    setup.setTracing(true);
    TagSchemaGenerator generator = new TagSchemaGenerator(setup);
    Map<String, String> symbolScopeMap = new LinkedHashMap<>();
    symbolScopeMap.put("Component", "NameScope");
    symbolScopeMap.put("Port", "NameScope");
    symbolScopeMap.put("ComponentInstance", "NameScope");
    symbolScopeMap.put("Connector", "ConnectorScope");
    generator.generate(Paths.get("CompPower"), Paths.get("src/test/resources/nfp/"), symbolScopeMap);
  }

  @Test
  public void testSizeTag() throws Exception {
    GeneratorSetup setup = new GeneratorSetup();
    //setup.setOutputDirectory(getPathFromRelativePath("src/test/java/").toFile());
    setup.setOutputDirectory(getPathFromRelativePath("src/test/resources/generator").toFile());
    setup.setTracing(true);
    TagSchemaGenerator generator = new TagSchemaGenerator(setup);
    Map<String, String> symbolScopeMap = new LinkedHashMap<>();
    symbolScopeMap.put("EMAComponentInstance", "NameScope");
    generator.generate(Paths.get("PhysicalTags"), Paths.get("src/test/resources/nfp"), symbolScopeMap);

  }

  @Test
  public void testOCL2JavaTag() throws Exception {
    GeneratorSetup setup = new GeneratorSetup();
    setup.setOutputDirectory(getPathFromRelativePath("src/test/resources/generator").toFile());
    setup.setTracing(true);
    TagSchemaGenerator generator = new TagSchemaGenerator(setup);
    Map<String, String> symbolScopeMap = new LinkedHashMap<>();
    symbolScopeMap.put("CDType", "NameScope");
    symbolScopeMap.put("OCLFile", "NameScope");
    generator.generate(Paths.get("OCL2JavaTags"), Paths.get("src/test/resources/tags"), symbolScopeMap);

  }
}
