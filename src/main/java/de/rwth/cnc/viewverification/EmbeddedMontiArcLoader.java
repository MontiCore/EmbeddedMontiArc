/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.java.lang.JavaDSLLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.rwth.cnc.model.*;

public class EmbeddedMontiArcLoader {

  public static ComponentSymbol loadComponentSymbol(String modelPath, String modelName) {
    String filePath = Paths.get(modelPath, modelName).toAbsolutePath().toString().replace('.', '/') + ".ema";
    assert new File(filePath).exists() : "File does not exist: " + filePath;
    Scope scope_model = createSymTab_EmbeddedMontiArc(modelPath);
    ComponentSymbol cmpSymbol = scope_model.<ComponentSymbol>resolve(modelName, ComponentSymbol.KIND).orElse(null);
    assert cmpSymbol != null : "\nResolve of " + modelName + " returned null!\nPath: " + modelPath;
    return cmpSymbol;
  }

  public static CnCArchitecture loadComponent(String modelPath, String modelName) {
    ComponentSymbol cmpSymbol = loadComponentSymbol(modelPath, modelName);
    CnCArchitecture cncArc = createCnCArchitecture(cmpSymbol);
    cncArc.setFileOrigin(Paths.get(modelPath, modelName.replace('.', '/')));
    cncArc.setPackageName(cmpSymbol.getPackageName());
    return cncArc;
  }

  private static CnCArchitecture createCnCArchitecture(ComponentSymbol cmpSymbol) {
    CnCArchitecture cncArc = new CnCArchitecture();
    cncArc.setName(cmpSymbol.getName());
    cncArc.setTopLevelComponentNames(Arrays.asList(cmpSymbol.getName()));

    extractComponent(cncArc, cmpSymbol, cmpSymbol.getName());
    extractSubComponentsRecursively(cncArc, cmpSymbol);
    extractComponentConnectionsRecursively(cncArc, cmpSymbol);

    return cncArc;
  }

  private static Scope createSymTab_EmbeddedMontiArc(String... modelPath) {
    ModelingLanguageFamily fam = new ModelingLanguageFamily();
    fam.addModelingLanguage(new EmbeddedMontiArcLanguage());
    fam.addModelingLanguage(new JavaDSLLanguage());
    fam.addModelingLanguage(new StreamLanguage());
    final ModelPath mp = new ModelPath(Paths.get("src/main/resources/defaultTypes"));
    for (String m : modelPath) {
      mp.addEntry(Paths.get(m));
    }
    GlobalScope scope = new GlobalScope(mp, fam);
    return scope;
  }

  private static void extractComponent(CnCArchitecture arc, ComponentSymbol cmpSymbol, String name) {
    Component cmp = new Component();
    cmp.setName(name);
    cmp.setComponentType(cmpSymbol.getName());
    cmp.setMarkedAtomic(cmpSymbol.isAtomic());
    //add all directly contained component names
    //    for (ComponentSymbol cs : cmpSymbol.getInnerComponents()) {
    for (ComponentInstanceSymbol cs : cmpSymbol.getSubComponents()) {
      cmp.addContainedComponent(cs.getName());
    }

    //add the ports of the component
    for (PortSymbol ps : cmpSymbol.getPorts()) {
      Port p = new Port();
      p.setDirection(ps.isIncoming() ? Direction.IN : Direction.OUT);
      p.setName(ps.getName());
      if (!ps.getTypeReference().getName().equals("SIUnitRangesType"))
        p.setType(ps.getTypeReference().getName());
      else
        p.setType(((SIUnitRangesSymbol) ps.getTypeReference().getReferencedSymbol()).getRange(0).toString());
      p.setTypeReference(Optional.of(ps.getTypeReference()));
      p.setComponent(cmp);
      assert !p.getName().contains(".") : "Portname is wrong!";
      cmp.addPort(p);
    }

    //effectors between all incoming and outgoing ports if component is atomic
    if (cmpSymbol.isAtomic()) {
      List<Port> incomingPorts = new LinkedList<>();
      List<Port> outgoingPorts = new LinkedList<>();

      for (Port p : cmp.getPorts()) {
        if (p.isIncoming())
          incomingPorts.add(p);
        else
          outgoingPorts.add(p);
      }

      for (Port ip : incomingPorts) {
        for (Port op : outgoingPorts) {
          Effector eff = new Effector();
          eff.setSender(cmp.getName());
          eff.setReceiver(cmp.getName());
          eff.setSenderPort(ip.getName());
          eff.setReceiverPort(op.getName());
          arc.addEffector(eff);
        }
      }
    }

    arc.addComponent(cmp);
  }

  private static void extractSubComponentsRecursively(CnCArchitecture arc, ComponentSymbol cmpSymbol) {
    for (ComponentInstanceSymbol cis : cmpSymbol.getSubComponents()) {
      extractComponent(arc, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(arc, cis.getComponentType().getReferencedSymbol());
    }
  }

  private static void extractComponentConnectionsRecursively(CnCArchitecture arc, ComponentSymbol componentSymbol) {
    //(1) get connectors
    //Debugging:
    Collection<ConnectorSymbol> colConSym = componentSymbol.getConnectors();
    for (ConnectorSymbol connectorSymbol : componentSymbol.getConnectors()) {
      Connection con = new Connection();

      String source = connectorSymbol.getSource();
      String target = connectorSymbol.getTarget();

      String spc = getPortComponent(source, componentSymbol.getName());
      String sp = getPortName(source);
      String tpc = getPortComponent(target, componentSymbol.getName());
      String tp = getPortName(target);

      con.setSender(spc);
      con.setSenderPort(sp);
      con.setReceiver(tpc);
      con.setReceiverPort(tp);

      arc.addConnection(con);
    }
    //(2) recurse
    for (ComponentInstanceSymbol cSymbol : componentSymbol.getSubComponents()) {
      extractComponentInstanceConnectionsRecursively(arc, cSymbol);
    }
  }

  private static void extractComponentInstanceConnectionsRecursively(CnCArchitecture arc, ComponentInstanceSymbol componentISymbol) {
    //(1) get connectors
    //Debugging:
    Collection<ConnectorSymbol> colConSym = componentISymbol.getComponentType().getConnectors();
    for (ConnectorSymbol connectorSymbol : componentISymbol.getComponentType().getConnectors()) {
      Connection con = new Connection();

      String source = connectorSymbol.getSource();
      String target = connectorSymbol.getTarget();

      String spc = getPortComponent(source, componentISymbol.getName());
      String sp = getPortName(source);
      String tpc = getPortComponent(target, componentISymbol.getName());
      String tp = getPortName(target);

      con.setSender(spc);
      con.setSenderPort(sp);
      con.setReceiver(tpc);
      con.setReceiverPort(tp);

      arc.addConnection(con);
    }
    //(2) recurse
    for (ComponentInstanceSymbol cSymbol : componentISymbol.getComponentType().getSubComponents()) {
      extractComponentInstanceConnectionsRecursively(arc, cSymbol);
    }
  }

  private static String getPortComponent(String str, String alternativeName) {
    if (str.contains(".")) {
      String[] split = str.split(("\\."));
      assert split.length == 2;
      return split[0];
    }

    return alternativeName;
  }

  private static String getPortName(String str) {
    if (str.contains(".")) {
      String[] split = str.split(("\\."));
      assert split.length == 2;
      return split[1];
    }
    return str;
  }
}
