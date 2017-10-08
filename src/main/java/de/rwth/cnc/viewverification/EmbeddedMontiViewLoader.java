package de.rwth.cnc.viewverification;

import java.nio.file.Paths;
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

  public static ComponentSymbol convertToEMVComponent(final CnCView view) {
    ComponentSymbol cmpSymbol = new ComponentSymbol(view.getName());

    List<String> topLevelComponents = view.getTopLevelComponentNames();
    for (String topLevelCmpName : topLevelComponents) {
      Component topComp = view.getComponent(topLevelCmpName);
      ComponentSymbol topCompSymb = createComponentSymbol(view, topComp);
      EMAComponentBuilder.addInnerComponent(cmpSymbol, topCompSymb);
    }

    return cmpSymbol;
  }

  private static final ResolvingFilter<ComponentSymbol> componentResolvingFilter =
      CommonResolvingFilter.create(ComponentSymbol.KIND);

  public static ViewSymbol convertToEMVView(final CnCView view) {
    ViewSymbol viewSymbol = new ViewSymbol(view.getName());

    List<String> topLevelComponents = view.getTopLevelComponentNames();
    for (String topLevelCmpName : topLevelComponents) {
      Component topComp = view.getComponent(topLevelCmpName);
      ComponentSymbol topCompSymb = createComponentSymbol(view, topComp);
      //EMAComponentBuilder.addInnerComponent(viewSymbol, topCompSymb);

      if(!viewSymbol.getSpannedScope().getResolvingFilters().contains(componentResolvingFilter)) {
        ((MutableScope) viewSymbol.getSpannedScope()).addResolver(componentResolvingFilter);
      }
      ((MutableScope) viewSymbol.getSpannedScope()).add(topCompSymb);
    }

    return viewSymbol;
  }

  private static ComponentSymbol createComponentSymbol(final CnCView view, final Component cmp) {
    ComponentSymbol componentSymbol = new ComponentSymbol(cmp.getName());
    componentSymbol.setMarkedAtomic(cmp.isMarkedAtomic());
    componentSymbol.setIsInterfaceComplete(cmp.isMarkedInterfaceComplete());

    for (String cName : cmp.getContainedComponents()) {
      Component c = view.getComponent(cName);
      ComponentSymbol innerComponent = createComponentSymbol(view, c);
      EMAComponentBuilder.addInnerComponent(componentSymbol, innerComponent);
    }

    for (Port p : cmp.getPorts()) {
      PortSymbol pSymbol = new PortSymbol(p.getName());
      pSymbol.setTypeReference(p.getTypeReference());
      pSymbol.setDirection(p.isIncoming());
      pSymbol.setEnclosingScope(componentSymbol.getSpannedScope().getAsMutableScope());
      EMAComponentBuilder.addPort(componentSymbol, pSymbol);
    }

    for (Connection con : view.getConnections()) {
      if (cmp.getContainedComponents().contains(con.getSender())) {
        ConnectorBuilder conB = new ConnectorBuilder();
        conB.setSource(con.getFullSender());
        conB.setTarget(con.getFullReceiver());
        EMAComponentBuilder.addConnector(componentSymbol, conB.build());
      }
    }

    for (Effector eff : view.getEffectors()) {
      if (cmp.getContainedComponents().contains(eff.getSender())) {
        EffectorBuilder effB = new EffectorBuilder();
        effB.setSource(eff.getFullSender());
        effB.setTarget(eff.getFullReceiver());
        EMAComponentBuilder.addEffector(componentSymbol, effB.build());
      }
    }

    return componentSymbol;

  }

  public static CnCView loadView(String viewPath, String viewName) {
    Scope scope_view = createSymTab_EmbeddedMontiView(viewPath);
    ViewSymbol viewSymbol = scope_view.<ViewSymbol>resolve(viewName, ViewSymbol.KIND).orElse(null);
    assert viewSymbol != null;

    CnCView cncView = createCnCView(viewSymbol);
    cncView.setFileOrigin(Paths.get(viewPath, viewName.replace('.', '/')));
    return cncView;
  }

  private static CnCView createCnCView(ViewSymbol viewSymbol) {
    CnCView cncView = new CnCView();
    cncView.setName(viewSymbol.getName());

    extractTopLevelComponentNames(cncView, viewSymbol);
    extractSubComponentsOfTopComponentsRecursively(cncView, viewSymbol);
    extractConnectionsRecursively(cncView, viewSymbol);
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

  private static void extractTopLevelComponentNames(CnCView cncView, ViewSymbol viewSymbol) {
    Collection<ComponentInstanceSymbol> ciscol = viewSymbol.getSubComponents();
    Collection<ComponentSymbol> cscol = viewSymbol.getInnerComponents();

    List<String> topComponentNames = new LinkedList<>();
    List<String> alreadyUsedComponents = new LinkedList<>();

    for (ComponentInstanceSymbol cis : ciscol) {
      topComponentNames.add(cis.getName());
      alreadyUsedComponents.add(cis.getComponentType().getReferencedSymbol().getName());
    }

    for (ComponentSymbol cs : cscol) {
      if (!alreadyUsedComponents.contains(cs.getName()))
        topComponentNames.add(cs.getName());
    }

    cncView.setTopLevelComponentNames(topComponentNames);
  }

  private static void extractSubComponentsRecursively(CnCView cncView, ComponentSymbol cmpSymbol) {
    for (ComponentInstanceSymbol cis : cmpSymbol.getSubComponents()) {
      extractComponent(cncView, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(cncView, cis.getComponentType().getReferencedSymbol());
    }
  }

  private static void extractSubComponentsOfTopComponentsRecursively(CnCView cncView, ViewSymbol viewSymbol) {
    List<String> alreadyUsedComponents = new LinkedList<>();
    for (ComponentInstanceSymbol cis : viewSymbol.getSubComponents()) {
      alreadyUsedComponents.add(cis.getComponentType().getReferencedSymbol().getName());
      extractComponent(cncView, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(cncView, cis.getComponentType().getReferencedSymbol());
    }
    for (ComponentSymbol cs : viewSymbol.getInnerComponents()) {
      if (!alreadyUsedComponents.contains(cs.getName())) {
        extractComponent(cncView, cs, cs.getName());
        extractSubComponentsRecursively(cncView, cs);
      }
    }
  }

  private static void extractSubComponentsOfTopComponentsRecursively(CnCView cncView, ComponentSymbol cmpSymbol) {
    List<String> alreadyUsedComponents = new LinkedList<>();
    for (ComponentInstanceSymbol cis : cmpSymbol.getSubComponents()) {
      alreadyUsedComponents.add(cis.getComponentType().getReferencedSymbol().getName());
      extractComponent(cncView, cis.getComponentType().getReferencedSymbol(), cis.getName());
      extractSubComponentsRecursively(cncView, cis.getComponentType().getReferencedSymbol());
    }
    for (ComponentSymbol cs : cmpSymbol.getInnerComponents()) {
      if (!alreadyUsedComponents.contains(cs.getName())) {
        extractComponent(cncView, cs, cs.getName());
        extractSubComponentsRecursively(cncView, cs);
      }
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
    for (EffectorSymbol effectorSymbol : componentSymbol.getEffectors()) {
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
