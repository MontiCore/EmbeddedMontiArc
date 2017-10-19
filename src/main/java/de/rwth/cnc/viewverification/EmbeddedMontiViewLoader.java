/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.cnc.viewverification;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.java.lang.JavaDSLLanguage;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.*;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.symboltable.*;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.rwth.cnc.model.*;

public class EmbeddedMontiViewLoader {

  private static final ResolvingFilter<ComponentSymbol> componentResolvingFilter = CommonResolvingFilter.create(ComponentSymbol.KIND);
  private static final ResolvingFilter<ComponentSymbol> connectorResolvingFilter = CommonResolvingFilter.create(ConnectorSymbol.KIND);
  private static final ResolvingFilter<ComponentSymbol> effectorResolvingFilter = CommonResolvingFilter.create(EffectorSymbol.KIND);

  public static ViewSymbol convertToEMVView(final CnCView view) {
    ViewSymbol viewSymbol = new ViewSymbol(view.getName());
    assert view.getPackageName() != null;
    viewSymbol.setPackageName(view.getPackageName());

    List<String> usedConnections = new ArrayList<>();
    List<String> topLevelComponents = view.getTopLevelComponentNames();
    for (String topLevelCmpName : topLevelComponents) {
      Component topComp = view.getComponent(topLevelCmpName);
      if (topComp == null)
        topComp = view.getComponent(VerificationHelper.uncapitalize(topLevelCmpName));
      assert topComp != null;
      ComponentSymbol topCompSymb = createComponentSymbol(view, topComp, topLevelCmpName, usedConnections);
      //EMAComponentBuilder.addInnerComponent(viewSymbol, topCompSymb);

      if (!viewSymbol.getSpannedScope().getResolvingFilters().contains(componentResolvingFilter)) {
        ((MutableScope) viewSymbol.getSpannedScope()).addResolver(componentResolvingFilter);
      }
      ((MutableScope) viewSymbol.getSpannedScope()).add(topCompSymb);
    }

    for (Connection con : view.getConnections()) {
      if (!usedConnections.contains(con.toString().toUpperCase())) {
        ConnectorSymbol conSym = createConnectorSymbol(con);
        addConnectorToView(viewSymbol, conSym);
      }
    }

    for (Effector eff : view.getEffectors()) {
      if (!usedConnections.contains(eff.toString().toUpperCase())) {
        EffectorSymbol effSym = createEffectorSymbol(eff);
        addEffectorToView(viewSymbol, effSym);
      }
    }

    return viewSymbol;
  }

  private static void addConnectorToView(ViewSymbol viewSymbol, ConnectorSymbol conSymbol) {
    if (!viewSymbol.getSpannedScope().getResolvingFilters().contains(connectorResolvingFilter)) {
      ((MutableScope) viewSymbol.getSpannedScope()).addResolver(connectorResolvingFilter);
    }
    ((MutableScope) viewSymbol.getSpannedScope()).add(conSymbol);
  }

  private static void addEffectorToView(ViewSymbol viewSymbol, EffectorSymbol effSymbol) {
    if (!viewSymbol.getSpannedScope().getResolvingFilters().contains(effectorResolvingFilter)) {
      ((MutableScope) viewSymbol.getSpannedScope()).addResolver(effectorResolvingFilter);
    }
    ((MutableScope) viewSymbol.getSpannedScope()).add(effSymbol);
  }

  private static ComponentSymbol createComponentSymbol(final CnCView view, final Component cmp, List<String> usedConnections) {
    return createComponentSymbol(view, cmp, VerificationHelper.capitalize(cmp.getName()), usedConnections);
  }

  private static ComponentSymbol createComponentSymbol(final CnCView view_p, final Component cmp, String name, List<String> usedConnections) {
    CnCView view = view_p.clone();
    ComponentSymbol componentSymbol = new ComponentSymbol(name);
    componentSymbol.setMarkedAtomic(cmp.isMarkedAtomic());
    componentSymbol.setIsInterfaceComplete(cmp.isMarkedInterfaceComplete());

    for (String cName : cmp.getContainedComponents()) {
      view.renameCmp(cName, VerificationHelper.capitalize(cName));
      Component c = view.getComponent(VerificationHelper.capitalize(cName));
      ComponentSymbol innerComponent = createComponentSymbol(view, c, usedConnections);
      EMAComponentBuilder.addInnerComponent(componentSymbol, innerComponent);
      innerComponent.setEnclosingScope((MutableScope) componentSymbol.getSpannedScope());
    }

    for (Port p : cmp.getPorts()) {
      PortSymbol pSymbol = new PortSymbol(p.getName());
      pSymbol.setTypeReference(p.getTypeReference());
      pSymbol.setDirection(p.isIncoming());
      pSymbol.setEnclosingScope(componentSymbol.getSpannedScope().getAsMutableScope());
      EMAComponentBuilder.addPort(componentSymbol, pSymbol);
    }

    for (Connection connection : view.getConnections()) {
      if (usedConnections.contains(connection.toString().toUpperCase()))
        continue;

      handleAddingConnectors(connection, cmp, componentSymbol, usedConnections);
    }

    for (Effector effect : view.getEffectors()) {
      if (usedConnections.contains(effect.toString().toUpperCase()))
        continue;

      handleAddingEffectors(effect, cmp, componentSymbol, usedConnections);
    }
    return componentSymbol;
  }

  private static void handleAddingConnectors(final Connection connection, final Component cmp, ComponentSymbol componentSymbol, List<String> usedConnections) {
    Connection con = connection.clone();
    if (con.getSender().equals(con.getReceiver()) && (con.getSender().equals(VerificationHelper.capitalize(cmp.getName())) || con.getSender().equals(VerificationHelper.uncapitalize(cmp.getName())))) {
      //inside of this component. sender = receiver = cmp
      usedConnections.add(connection.toString().toUpperCase());
      con.setSender("");
      con.setReceiver("");

      ConnectorSymbol conB = createConnectorSymbol(con);
      EMAComponentBuilder.addConnector(componentSymbol, conB);
    }
    else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(con.getSender()))) {
      usedConnections.add(connection.toString().toUpperCase());
      con.setSender(VerificationHelper.uncapitalize(con.getSender()));
      if (!cmp.getName().equals(con.getReceiver()))
        con.setReceiver(VerificationHelper.uncapitalize(con.getReceiver()));
      else
        con.setReceiver("");
      ConnectorSymbol conB = createConnectorSymbol(con);
      EMAComponentBuilder.addConnector(componentSymbol, conB);
    }
    else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(con.getReceiver()))) {
      usedConnections.add(connection.toString().toUpperCase());
      con.setReceiver(VerificationHelper.uncapitalize(con.getReceiver()));
      if (!cmp.getName().equals(con.getSender()))
        con.setSender(VerificationHelper.uncapitalize(con.getSender()));
      else
        con.setSender("");
      ConnectorSymbol conB = createConnectorSymbol(con);
      EMAComponentBuilder.addConnector(componentSymbol, conB);
    }
  }

  private static void handleAddingEffectors(final Effector effect, final Component cmp, ComponentSymbol componentSymbol, List<String> usedConnections) {
    Effector eff = effect.clone();
    if (eff.getSender().equals(eff.getReceiver()) && (eff.getSender().equals(VerificationHelper.capitalize(cmp.getName())) || eff.getSender().equals(VerificationHelper.uncapitalize(cmp.getName())))) {
      //inside of this component. sender = receiver = cmp
      usedConnections.add(effect.toString().toUpperCase());
      eff.setSender("");
      eff.setReceiver("");

      EffectorSymbol effB = createEffectorSymbol(eff);
      EMAComponentBuilder.addEffector(componentSymbol, effB);
    }
    else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(eff.getSender()))) {
      usedConnections.add(effect.toString().toUpperCase());
      eff.setSender(VerificationHelper.uncapitalize(eff.getSender()));
      if (!cmp.getName().equals(eff.getReceiver()))
        eff.setReceiver(VerificationHelper.uncapitalize(eff.getReceiver()));
      else
        eff.setReceiver("");
      EffectorSymbol effB = createEffectorSymbol(eff);
      EMAComponentBuilder.addEffector(componentSymbol, effB);
    }
    else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(eff.getReceiver()))) {
      usedConnections.add(effect.toString().toUpperCase());
      eff.setReceiver(VerificationHelper.uncapitalize(eff.getReceiver()));
      if (!cmp.getName().equals(eff.getSender()))
        eff.setSender(VerificationHelper.uncapitalize(eff.getSender()));
      else
        eff.setSender("");
      EffectorSymbol effB = createEffectorSymbol(eff);
      EMAComponentBuilder.addEffector(componentSymbol, effB);
    }

  }

  private static ConnectorSymbol createConnectorSymbol(Connection con) {
    ConnectorBuilder conB = new ConnectorBuilder();
    conB.setSource(con.getFullSender());
    conB.setTarget(con.getFullReceiver());
    return conB.build();
  }

  private static EffectorSymbol createEffectorSymbol(Effector eff) {
    EffectorBuilder effB = new EffectorBuilder();
    effB.setSource(eff.getFullSender());
    effB.setTarget(eff.getFullReceiver());
    return effB.build();
  }

  public static ViewSymbol loadViewSymbol(String viewPath, String viewName) {
    Scope scope_view = createSymTab_EmbeddedMontiView(viewPath);
    ViewSymbol viewSymbol = scope_view.<ViewSymbol>resolve(viewName, ViewSymbol.KIND).orElse(null);
    assert viewSymbol != null : "\nResolve of " + viewName + " returned null!\nPath: " + viewPath;
    return viewSymbol;
  }

  public static CnCView loadView(String viewPath, String viewName) {
    ViewSymbol viewSymbol = loadViewSymbol(viewPath, viewName);
    CnCView cncView = createCnCView(viewSymbol);
    cncView.setFileOrigin(Paths.get(viewPath, viewName.replace('.', '/')));
    assert viewSymbol.getPackageName() != null;
    return cncView;
  }

  private static Scope createSymTab_EmbeddedMontiView(String... modelPath) {
    ModelingLanguageFamily fam = new ModelingLanguageFamily();
    fam.addModelingLanguage(new EmbeddedMontiViewLanguage());
    fam.addModelingLanguage(new JavaDSLLanguage());
    fam.addModelingLanguage(new StreamLanguage());
    final ModelPath mp = new ModelPath(Paths.get("src/main/resources/defaultTypes"));
    for (String m : modelPath) {
      mp.addEntry(Paths.get(m));
    }
    GlobalScope scope = new GlobalScope(mp, fam);
    return scope;
  }

  public static CnCView createCnCView(ViewSymbol viewSymbol) {
    CnCView cncView = new CnCView();
    cncView.setName(viewSymbol.getName());
    cncView.setPackageName(viewSymbol.getPackageName());

    extractTopLevelComponents(cncView, viewSymbol);
    extractSubComponentsOfTopComponentsRecursively(cncView, viewSymbol);
    extractConnectionsRecursively(cncView, viewSymbol);
    return cncView;
  }

  private static void extractTopLevelComponents(CnCView cncView, ViewSymbol viewSymbol) {
    //    Collection<ComponentInstanceSymbol> ciscol = viewSymbol.getSubComponents();
    Collection<ComponentSymbol> cscol = viewSymbol.getInnerComponents();

    List<String> topComponentNames = new LinkedList<>();
    //    List<String> alreadyUsedComponents = new LinkedList<>();

    //    for (ComponentInstanceSymbol cis : ciscol) {
    //      topComponentNames.add(cis.getName());
    //      alreadyUsedComponents.add(cis.getComponentType().getReferencedSymbol().getName());
    //    }

    for (ComponentSymbol cs : cscol) {
      //      if (!alreadyUsedComponents.contains(cs.getName()))
      topComponentNames.add(cs.getName());
      assert Character.isUpperCase(cs.getName().charAt(0));
      extractComponent(cncView, cs, cs.getName());
    }

    cncView.setTopLevelComponentNames(topComponentNames);
    //    for (ComponentSymbol cs : viewSymbol.getInnerComponents()) {
    //      if (!alreadyUsedComponents.contains(cs.getName())) {
    //        assert Character.isUpperCase(cs.getName().charAt(0));
    //        extractComponent(cncView, cs, cs.getName());
    //        extractSubComponentsRecursively(cncView, cs);
    //      }
    //    }
  }

  private static void extractSubComponentsRecursively(CnCView cncView, ComponentSymbol cmpSymbol) {
    for (ComponentInstanceSymbol cis : cmpSymbol.getSubComponents()) {
      extractComponent(cncView, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(cncView, cis.getComponentType().getReferencedSymbol());
    }
  }

  private static void extractSubComponentsOfTopComponentsRecursively(CnCView cncView, ViewSymbol viewSymbol) {
    for (ComponentSymbol cs : viewSymbol.getInnerComponents()) {
      extractSubComponentsOfTopComponentsRecursively(cncView, cs);
    }
  }

  private static void extractSubComponentsOfTopComponentsRecursively(CnCView cncView, ComponentSymbol cmpSymbol) {
    for (ComponentInstanceSymbol cis : cmpSymbol.getSubComponents()) {
      extractComponent(cncView, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(cncView, cis.getComponentType().getReferencedSymbol());
    }
  }

  private static void extractConnectionsRecursively(CnCView cncView, ViewSymbol viewSymbol) {
    //(1) get connectors
    for (ConnectorSymbol connectorSymbol : viewSymbol.getConnectors()) {
      Connection con = new Connection();

      //sps = sourceportsymbol
      PortSymbol sps = connectorSymbol.getSourcePort();
      PortSymbol tps = connectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

      if (sps != null) {
        spc = connectorSymbol.getSourcePort().getComponent().get().getName();
        sp = connectorSymbol.getSourcePort().getName();
      }
      else {
        spc = connectorSymbol.getSource();
        sp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (spc.contains(".")) {
          String[] spcsplit = spc.split("\\.");
          if (spcsplit.length == 2) {
            spc = spcsplit[0];
            sp = spcsplit[1];
          }
        }
      }

      if (tps != null) {
        tpc = connectorSymbol.getTargetPort().getComponent().get().getName();
        tp = connectorSymbol.getTargetPort().getName();
      }
      else {
        tpc = connectorSymbol.getTarget();
        tp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (tpc.contains(".")) {
          String[] tpcsplit = tpc.split("\\.");
          if (tpcsplit.length == 2) {
            tpc = tpcsplit[0];
            tp = tpcsplit[1];
          }
        }
      }

      con.setSender(spc);
      con.setSenderPort(sp);
      con.setReceiver(tpc);
      con.setReceiverPort(tp);

      cncView.addConnection(con);
    }
    //(2) get effectors
    for (EffectorSymbol effectorSymbol : viewSymbol.getEffectors()) {
      Effector eff = new Effector();

      //sps = sourceportsymbol
      PortSymbol sps = effectorSymbol.getSourcePort();
      PortSymbol tps = effectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

      if (sps != null) {
        spc = effectorSymbol.getSourcePort().getComponent().get().getName();
        sp = effectorSymbol.getSourcePort().getName();
      }
      else {
        spc = effectorSymbol.getSource();
        sp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (spc.contains(".")) {
          String[] spcsplit = spc.split("\\.");
          if (spcsplit.length == 2) {
            spc = spcsplit[0];
            sp = spcsplit[1];
          }
        }
      }

      if (tps != null) {
        tpc = effectorSymbol.getTargetPort().getComponent().get().getName();
        tp = effectorSymbol.getTargetPort().getName();
      }
      else {
        tpc = effectorSymbol.getTarget();
        tp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (tpc.contains(".")) {
          String[] tpcsplit = tpc.split("\\.");
          if (tpcsplit.length == 2) {
            tpc = tpcsplit[0];
            tp = tpcsplit[1];
          }
        }
      }

      eff.setSender(spc);
      eff.setSenderPort(sp);
      eff.setReceiver(tpc);
      eff.setReceiverPort(tp);

      cncView.addEffector(eff);
    }
    //(3) recurse
    for (ComponentSymbol cSymbol : viewSymbol.getInnerComponents()) {
      extractConnectionsRecursively(cncView, cSymbol);
    }
  }

  private static void extractConnectionsRecursively(CnCView cncView, ComponentSymbol componentSymbol) {
    //(1) get connectors
    for (ConnectorSymbol connectorSymbol : componentSymbol.getConnectors()) {
      Connection con = new Connection();

      //sps = sourceportsymbol
      PortSymbol sps = connectorSymbol.getSourcePort();
      PortSymbol tps = connectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

//      if (sps != null) {
//        spc = connectorSymbol.getSourcePort().getComponent().get().getName();
//        sp = connectorSymbol.getSourcePort().getName();
//      }
//      else
        {
        spc = connectorSymbol.getSource();
        sp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (spc.contains(".")) {
          String[] spcsplit = spc.split("\\.");
          if (spcsplit.length == 2) {
            spc = spcsplit[0];
            sp = spcsplit[1];
          }
        }
      }

//      if (tps != null) {
//        tpc = connectorSymbol.getTargetPort().getComponent().get().getName();
//        tp = connectorSymbol.getTargetPort().getName();
//      }
//      else
        {
        tpc = connectorSymbol.getTarget();
        tp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (tpc.contains(".")) {
          String[] tpcsplit = tpc.split("\\.");
          if (tpcsplit.length == 2) {
            tpc = tpcsplit[0];
            tp = tpcsplit[1];
          }
        }
      }

      con.setSender(spc);
      con.setSenderPort(sp);
      con.setReceiver(tpc);
      con.setReceiverPort(tp);

      cncView.addConnection(con);
    }
    //(2) get effectors
    for (EffectorSymbol effectorSymbol : componentSymbol.getEffectors()) {
      Effector eff = new Effector();

      //sps = sourceportsymbol
      PortSymbol sps = effectorSymbol.getSourcePort();
      PortSymbol tps = effectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

//      if (sps != null) {
//        spc = effectorSymbol.getSourcePort().getComponent().get().getName();
//        sp = effectorSymbol.getSourcePort().getName();
//      }
//      else
        {
        spc = effectorSymbol.getSource();
        sp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (spc.contains(".")) {
          String[] spcsplit = spc.split("\\.");
          if (spcsplit.length == 2) {
            spc = spcsplit[0];
            sp = spcsplit[1];
          }
        }
      }

//      if (tps != null) {
//        tpc = effectorSymbol.getTargetPort().getComponent().get().getName();
//        tp = effectorSymbol.getTargetPort().getName();
//      }
//      else
        {
        tpc = effectorSymbol.getTarget();
        tp = null;

        //fix some unknown error where getSourcePort()/Target doesnt give the port back
        if (tpc.contains(".")) {
          String[] tpcsplit = tpc.split("\\.");
          if (tpcsplit.length == 2) {
            tpc = tpcsplit[0];
            tp = tpcsplit[1];
          }
        }
      }

      eff.setSender(spc);
      eff.setSenderPort(sp);
      eff.setReceiver(tpc);
      eff.setReceiverPort(tp);

      cncView.addEffector(eff);
    }
    //(3) recurse
    for (ComponentSymbol cSymbol : componentSymbol.getInnerComponents()) {
      extractConnectionsRecursively(cncView, cSymbol);
    }
  }

  private static void extractComponent(CnCView view, ComponentSymbol cmpSymbol, String name) {
    Component cmp = new Component();
    cmp.setName(name);
    cmp.setComponentType(cmpSymbol.getName());
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
      p.setType(ps.getTypeName());
      p.setTypeReference(ps.getTypeReference());
      p.setComponent(cmp);
      cmp.addPort(p);
    }

    cmp.setMarkedInterfaceComplete(cmpSymbol.isInterfaceComplete());
    cmp.setMarkedAtomic(cmpSymbol.isMarkedAtomic());

    view.addComponent(cmp);
  }
}
