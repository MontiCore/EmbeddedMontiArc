/* (c) https://github.com/MontiCore/monticore */
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
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.symboltable.*;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.rwth.cnc.model.*;

public class EmbeddedMontiViewLoader {

  private static final ResolvingFilter<ViewComponentSymbol> componentResolvingFilter = CommonResolvingFilter.create(ViewComponentSymbol.KIND);
  private static final ResolvingFilter<ViewComponentSymbol> connectorResolvingFilter = CommonResolvingFilter.create(ViewConnectorSymbol.KIND);
  private static final ResolvingFilter<ViewComponentSymbol> effectorResolvingFilter = CommonResolvingFilter.create(ViewEffectorSymbol.KIND);

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
      ViewComponentSymbol topCompSymb = createViewComponentSymbol(view, topComp, topLevelCmpName, usedConnections);
      //EMAComponentBuilder.addInnerComponent(viewSymbol, topCompSymb);

      if (!viewSymbol.getSpannedScope().getResolvingFilters().contains(componentResolvingFilter)) {
        ((MutableScope) viewSymbol.getSpannedScope()).addResolver(componentResolvingFilter);
      }
      ((MutableScope) viewSymbol.getSpannedScope()).add(topCompSymb);
    }

    for (Connection con : view.getConnections()) {
      if (!usedConnections.contains(con.toString().toUpperCase())) {
        ViewConnectorSymbol conSym = createViewConnectorSymbol(con);
        addConnectorToView(viewSymbol, conSym);
      }
    }

    for (Effector eff : view.getEffectors()) {
      if (!usedConnections.contains(eff.toString().toUpperCase())) {
        ViewEffectorSymbol effSym = createViewEffectorSymbol(eff);
        addEffectorToView(viewSymbol, effSym);
      }
    }

    return viewSymbol;
  }

  private static void addConnectorToView(ViewSymbol viewSymbol, ViewConnectorSymbol conSymbol) {
    if (!viewSymbol.getSpannedScope().getResolvingFilters().contains(connectorResolvingFilter)) {
      ((MutableScope) viewSymbol.getSpannedScope()).addResolver(connectorResolvingFilter);
    }
    ((MutableScope) viewSymbol.getSpannedScope()).add(conSymbol);
  }

  private static void addEffectorToView(ViewSymbol viewSymbol, ViewEffectorSymbol effSymbol) {
    if (!viewSymbol.getSpannedScope().getResolvingFilters().contains(effectorResolvingFilter)) {
      ((MutableScope) viewSymbol.getSpannedScope()).addResolver(effectorResolvingFilter);
    }
    ((MutableScope) viewSymbol.getSpannedScope()).add(effSymbol);
  }

  private static ViewComponentSymbol createViewComponentSymbol(final CnCView view, final Component cmp, List<String> usedConnections) {
    return createViewComponentSymbol(view, cmp, VerificationHelper.capitalize(cmp.getName()), usedConnections);
  }

  private static ViewComponentSymbol createViewComponentSymbol(final CnCView view_p, final Component cmp, String name, List<String> usedConnections) {
    CnCView view = view_p.clone();
    ViewComponentSymbol ViewComponentSymbol = new ViewComponentSymbol(name);
    ViewComponentSymbol.setMarkedAtomic(cmp.isMarkedAtomic());
    ViewComponentSymbol.setIsInterfaceComplete(cmp.isMarkedInterfaceComplete());

    for (String cName : cmp.getContainedComponents()) {
      view.renameCmp(cName, VerificationHelper.capitalize(cName));
      Component c = view.getComponent(VerificationHelper.capitalize(cName));
      ViewComponentSymbol innerComponent = createViewComponentSymbol(view, c, usedConnections);
      EMAComponentBuilder.addInnerComponent(ViewComponentSymbol, innerComponent);
      innerComponent.setEnclosingScope((MutableScope) ViewComponentSymbol.getSpannedScope());
    }

    for (Port p : cmp.getPorts()) {
      ViewPortSymbol pSymbol = new ViewPortSymbol(p.getName());
      pSymbol.setTypeReference(p.getTypeReference());
      pSymbol.setDirection(p.isIncoming());
      pSymbol.setEnclosingScope(ViewComponentSymbol.getSpannedScope().getAsMutableScope());
      EMAComponentBuilder.addPort(ViewComponentSymbol, pSymbol);
    }

    for (Connection connection : view.getConnections()) {
      if (usedConnections.contains(connection.toString().toUpperCase()))
        continue;

      handleAddingConnectors(connection, cmp, ViewComponentSymbol, usedConnections);
    }

    for (Effector effect : view.getEffectors()) {
      if (usedConnections.contains(effect.toString().toUpperCase()))
        continue;

      handleAddingEffectors(effect, cmp, ViewComponentSymbol, usedConnections);
    }
    return ViewComponentSymbol;
  }

  private static void handleAddingConnectors(final Connection connection, final Component cmp, ViewComponentSymbol ViewComponentSymbol, List<String> usedConnections) {
    Connection con = connection.clone();
    if (con.getSender().equals(con.getReceiver()) && (con.getSender().equals(VerificationHelper.capitalize(cmp.getName())) || con.getSender().equals(VerificationHelper.uncapitalize(cmp.getName())))) {
      //inside of this component. sender = receiver = cmp
      usedConnections.add(connection.toString().toUpperCase());
      con.setSender("");
      con.setReceiver("");

      ViewConnectorSymbol conB = createViewConnectorSymbol(con);
      EMAComponentBuilder.addConnector(ViewComponentSymbol, conB);
    } else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(con.getSender()))) {
      usedConnections.add(connection.toString().toUpperCase());
      con.setSender(VerificationHelper.uncapitalize(con.getSender()));
      if (!cmp.getName().equals(con.getReceiver()))
        con.setReceiver(VerificationHelper.uncapitalize(con.getReceiver()));
      else
        con.setReceiver("");
      ViewConnectorSymbol conB = createViewConnectorSymbol(con);
      EMAComponentBuilder.addConnector(ViewComponentSymbol, conB);
    } else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(con.getReceiver()))) {
      usedConnections.add(connection.toString().toUpperCase());
      con.setReceiver(VerificationHelper.uncapitalize(con.getReceiver()));
      if (!cmp.getName().equals(con.getSender()))
        con.setSender(VerificationHelper.uncapitalize(con.getSender()));
      else
        con.setSender("");
      ViewConnectorSymbol conB = createViewConnectorSymbol(con);
      EMAComponentBuilder.addConnector(ViewComponentSymbol, conB);
    }
  }

  private static void handleAddingEffectors(final Effector effect, final Component cmp, ViewComponentSymbol ViewComponentSymbol, List<String> usedConnections) {
    Effector eff = effect.clone();
    if (eff.getSender().equals(eff.getReceiver()) && (eff.getSender().equals(VerificationHelper.capitalize(cmp.getName())) || eff.getSender().equals(VerificationHelper.uncapitalize(cmp.getName())))) {
      //inside of this component. sender = receiver = cmp
      usedConnections.add(effect.toString().toUpperCase());
      eff.setSender("");
      eff.setReceiver("");

      ViewEffectorSymbol effB = createViewEffectorSymbol(eff);
      EMAComponentBuilder.addEffector(ViewComponentSymbol, effB);
    } else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(eff.getSender()))) {
      usedConnections.add(effect.toString().toUpperCase());
      eff.setSender(VerificationHelper.uncapitalize(eff.getSender()));
      if (!cmp.getName().equals(eff.getReceiver()))
        eff.setReceiver(VerificationHelper.uncapitalize(eff.getReceiver()));
      else
        eff.setReceiver("");
      ViewEffectorSymbol effB = createViewEffectorSymbol(eff);
      EMAComponentBuilder.addEffector(ViewComponentSymbol, effB);
    } else if (cmp.getContainedComponents().contains(VerificationHelper.uncapitalize(eff.getReceiver()))) {
      usedConnections.add(effect.toString().toUpperCase());
      eff.setReceiver(VerificationHelper.uncapitalize(eff.getReceiver()));
      if (!cmp.getName().equals(eff.getSender()))
        eff.setSender(VerificationHelper.uncapitalize(eff.getSender()));
      else
        eff.setSender("");
      ViewEffectorSymbol effB = createViewEffectorSymbol(eff);
      EMAComponentBuilder.addEffector(ViewComponentSymbol, effB);
    }

  }

  private static ViewConnectorSymbol createViewConnectorSymbol(Connection con) {
    ViewConnectorBuilder conB = new ViewConnectorBuilder();
    conB.setSource(con.getFullSender());
    conB.setTarget(con.getFullReceiver());
    return conB.build();
  }

  private static ViewEffectorSymbol createViewEffectorSymbol(Effector eff) {
    ViewEffectorBuilder effB = new ViewEffectorBuilder();
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
    //    Collection<ViewComponentInstanceSymbol> ciscol = viewSymbol.getSubComponents();
    Collection<ViewComponentSymbol> cscol = viewSymbol.getInnerComponents();

    List<String> topComponentNames = new LinkedList<>();
    //    List<String> alreadyUsedComponents = new LinkedList<>();

    //    for (ViewComponentInstanceSymbol cis : ciscol) {
    //      topComponentNames.add(cis.getName());
    //      alreadyUsedComponents.add(cis.getComponentType().getReferencedSymbol().getName());
    //    }

    for (ViewComponentSymbol cs : cscol) {
      //      if (!alreadyUsedComponents.contains(cs.getName()))
      topComponentNames.add(cs.getName());
      assert Character.isUpperCase(cs.getName().charAt(0));
      extractComponent(cncView, cs, cs.getName());
    }

    cncView.setTopLevelComponentNames(topComponentNames);
    //    for (ViewComponentSymbol cs : viewSymbol.getInnerComponents()) {
    //      if (!alreadyUsedComponents.contains(cs.getName())) {
    //        assert Character.isUpperCase(cs.getName().charAt(0));
    //        extractComponent(cncView, cs, cs.getName());
    //        extractSubComponentsRecursively(cncView, cs);
    //      }
    //    }
  }

  private static void extractSubComponentsRecursively(CnCView cncView, ViewComponentSymbol cmpSymbol) {
    for (ViewComponentInstanceSymbol cis : cmpSymbol.getSubComponents()) {
      extractComponent(cncView, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(cncView, cis.getComponentType().getReferencedSymbol());
    }
  }

  private static void extractSubComponentsOfTopComponentsRecursively(CnCView cncView, ViewSymbol viewSymbol) {
    for (ViewComponentSymbol cs : viewSymbol.getInnerComponents()) {
      extractSubComponentsOfTopComponentsRecursively(cncView, cs);
    }
  }

  private static void extractSubComponentsOfTopComponentsRecursively(CnCView cncView, ViewComponentSymbol cmpSymbol) {
    for (ViewComponentInstanceSymbol cis : cmpSymbol.getSubComponents()) {
      extractComponent(cncView, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(cncView, cis.getComponentType().getReferencedSymbol());
    }
  }

  private static void extractConnectionsRecursively(CnCView cncView, ViewSymbol viewSymbol) {
    //(1) get connectors
    getConnectors(viewSymbol, cncView);

    //(2) get effectors
    getEffectors(viewSymbol, cncView);


    //(3) recurse
    for (ViewComponentSymbol cSymbol : viewSymbol.getInnerComponents()) {
      extractConnectionsRecursively(cncView, cSymbol);
    }
  }

  private static void extractConnectionsRecursively(CnCView cncView, ViewComponentSymbol viewComponentSymbol) {
    getConnectors(viewComponentSymbol, cncView);
    getEffectors(viewComponentSymbol, cncView);

    //(3) recurse
    for (ViewComponentSymbol cSymbol : viewComponentSymbol.getInnerComponents()) {
      extractConnectionsRecursively(cncView, cSymbol);
    }
  }

  private static void getConnectors(ViewSymbol viewSymbol, CnCView cncView) {
    for (ViewConnectorSymbol viewConnectorSymbol : viewSymbol.getConnectors()) {
      Connection con = new Connection();

      //sps = sourceViewPortSymbol
      ViewPortSymbol sps = viewConnectorSymbol.getSourcePort();
      ViewPortSymbol tps = viewConnectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

      if (sps != null) {
        spc = viewConnectorSymbol.getSourcePort().getComponent().get().getName();
        sp = viewConnectorSymbol.getSourcePort().getName();
      } else {
        spc = viewConnectorSymbol.getSource();
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
        tpc = viewConnectorSymbol.getTargetPort().getComponent().get().getName();
        tp = viewConnectorSymbol.getTargetPort().getName();
      } else {
        tpc = viewConnectorSymbol.getTarget();
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
  }

  private static void getEffectors(ViewSymbol viewSymbol, CnCView cncView) {
    for (ViewEffectorSymbol ViewEffectorSymbol : viewSymbol.getEffectors()) {
      Effector eff = new Effector();

      //sps = sourceViewPortSymbol
      ViewPortSymbol sps = ViewEffectorSymbol.getSourcePort();
      ViewPortSymbol tps = ViewEffectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

      if (sps != null) {
        spc = ViewEffectorSymbol.getSourcePort().getComponent().get().getName();
        sp = ViewEffectorSymbol.getSourcePort().getName();
      } else {
        spc = ViewEffectorSymbol.getSource();
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
        tpc = ViewEffectorSymbol.getTargetPort().getComponent().get().getName();
        tp = ViewEffectorSymbol.getTargetPort().getName();
      } else {
        tpc = ViewEffectorSymbol.getTarget();
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

      // TODO: Debug for WCET.emv effector (controlSignalsIn.null -> dataSaveInternalOut.null)
      eff.setSender(spc);
      eff.setSenderPort(sp);
      eff.setReceiver(tpc);
      eff.setReceiverPort(tp);

      cncView.addEffector(eff);
    }
  }

  private static void getConnectors(ViewComponentSymbol viewComponentSymbol, CnCView cncView) {
    //(1) get connectors
    for (ViewConnectorSymbol viewConnectorSymbol : viewComponentSymbol.getConnectors()) {
      Connection con = new Connection();

      //sps = sourceViewPortSymbol
      ViewPortSymbol sps = viewConnectorSymbol.getSourcePort();
      ViewPortSymbol tps = viewConnectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

//      if (sps != null) {
//        spc = ViewConnectorSymbol.getSourcePort().getComponent().get().getName();
//        sp = ViewConnectorSymbol.getSourcePort().getName();
//      }
//      else
      {
        spc = viewConnectorSymbol.getSource();
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
//        tpc = ViewConnectorSymbol.getTargetPort().getComponent().get().getName();
//        tp = ViewConnectorSymbol.getTargetPort().getName();
//      }
//      else
      {
        tpc = viewConnectorSymbol.getTarget();
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
  }

  private static void getEffectors(ViewComponentSymbol viewComponentSymbol, CnCView cncView) {
    //(2) get effectors
    for (ViewEffectorSymbol ViewEffectorSymbol : viewComponentSymbol.getEffectors()) {
      Effector eff = new Effector();

      //sps = sourceViewPortSymbol
      ViewPortSymbol sps = ViewEffectorSymbol.getSourcePort();
      ViewPortSymbol tps = ViewEffectorSymbol.getTargetPort();
      //spc = sourceportcomponent
      String spc = null, tpc = null;
      //sp  = sourceport
      String sp = null, tp = null;

//      if (sps != null) {
//        spc = ViewEffectorSymbol.getSourcePort().getComponent().get().getName();
//        sp = ViewEffectorSymbol.getSourcePort().getName();
//      }
//      else
      {
        spc = ViewEffectorSymbol.getSource();
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
//        tpc = ViewEffectorSymbol.getTargetPort().getComponent().get().getName();
//        tp = ViewEffectorSymbol.getTargetPort().getName();
//      }
//      else
      {
        tpc = ViewEffectorSymbol.getTarget();
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
  }

  private static void extractComponent(CnCView view, ViewComponentSymbol cmpSymbol, String name) {
    Component cmp = new Component();
    cmp.setName(name);
    cmp.setComponentType(cmpSymbol.getName());
    //add all directly contained component names
    //    for (ViewComponentSymbol cs : cmpSymbol.getInnerComponents()) {
    for (ViewComponentInstanceSymbol cs : cmpSymbol.getSubComponents()) {
      cmp.addContainedComponent(cs.getName());
    }
    //add the ports of the component
    for (ViewPortSymbol ps : cmpSymbol.getPorts()) {
      Port p = new Port();
      p.setDirection(ps.isIncoming() ? Direction.IN : Direction.OUT);
      p.setName(ps.getName());
      p.setType(ps.getTypeName());
      if (!ps.getTypeName().equals("SIUnitRangesType"))
        p.setType(ps.getTypeName());
      else
        p.setType(((SIUnitRangesSymbol) ps.getTypeReference().get().getReferencedSymbol()).getRange(0).toString());
      p.setTypeReference(ps.getTypeReference());
      p.setComponent(cmp);
      cmp.addPort(p);
    }

    cmp.setMarkedInterfaceComplete(cmpSymbol.isInterfaceComplete());
    cmp.setMarkedAtomic(cmpSymbol.isMarkedAtomic());

    view.addComponent(cmp);
  }
}
