/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import static javolution.testing.TestContext.assertEquals;

import javax.swing.text.View;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.*;
import de.rwth.cnc.model.CnCArchitecture;
import de.rwth.cnc.model.CnCView;
import de.rwth.cnc.model.Component;
import de.rwth.cnc.model.Effector;
import de.rwth.cnc.viewverification.EmbeddedMontiArcLoader;
import de.rwth.cnc.viewverification.EmbeddedMontiViewLoader;
import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.Inconsistency;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import de.rwth.cnc.viewverification.witness.WitnessGeneratorHelper;

public class TestHelper {

  public static void checkModelViewWitnessResult(String modelPath, String modelName, String viewPath, String viewName, String expectedWitnessPath, String expectedWitnessName) {

    CnCArchitecture model = EmbeddedMontiArcLoader.loadComponent(modelPath, modelName);
    CnCView view = EmbeddedMontiViewLoader.loadView(viewPath, viewName);
    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(model, view, true, true);
    //assert inconsistencies.size() < 2 : "More than one witness is being generated!";

    ViewSymbol expectedWitness = EmbeddedMontiViewLoader.loadViewSymbol(expectedWitnessPath, expectedWitnessName);
    //    CnCView expectedWitness = EmbeddedMontiViewLoader.loadView(expectedWitnessPath, expectedWitnessName);
    ViewSymbol generatedWitness;
    //    CnCView generatedWitness = ViewVerificator.getLastGeneratedWitness();

    if (inconsistencies.size() == 0) {
      Path generatedWitnessPath = Paths.get("target", "generated-witnesses", "positive", view.getName());
      generatedWitness = EmbeddedMontiViewLoader.loadViewSymbol(generatedWitnessPath.toString(), view.getPackageName() + ".Witness_" + model.getName());
    }
    else {
      generatedWitness = getNegativeWitness(inconsistencies.get(0), view.getPackageName());
    }

    //generatedWitness = getCopyWithNewName(generatedWitness, expectedWitness.getName());

    assertEquality(generatedWitness, expectedWitness);
  }

  private static ViewSymbol getNegativeWitness(InconsistencyItem incItem, String packageName) {
    // get full path
    String pathStringFull = incItem.getJustificationFileName();
    // get info
    String[] pathArray = pathStringFull.replaceAll("\\.", "/").split("[/\\\\]");
    int packageNameLength = packageName.split("\\.").length + 2; // +2 adds filename and fileending
    // get file name
    String fileName = pathArray[pathArray.length - 2];
    String viewNameFull = packageName + "." + fileName;
    // extract path without full name
    String pathName = "";
    for (int i = 0; i < pathArray.length - packageNameLength; i++) {
      pathName += pathArray[i] + "/";
    }
    // load view
    ViewSymbol v = EmbeddedMontiViewLoader.loadViewSymbol(pathName, viewNameFull);
    return v;
  }

  private static ViewSymbol getCopyWithNewName(ViewSymbol view, String name) {
    CnCView newCnCView = EmbeddedMontiViewLoader.createCnCView(view);
    newCnCView.setName(name);
    ViewSymbol newView = EmbeddedMontiViewLoader.convertToEMVView(newCnCView);
    assert view.getAstNode().isPresent();
    newView.setAstNode(view.getAstNode().get());
    return newView;
  }

  public static void assertEqualityDeepEquals(ViewSymbol witness, ViewSymbol expectedWitness) {
    assert expectedWitness.getAstNode().isPresent();
    assert witness.getAstNode().isPresent();
    assert expectedWitness.getAstNode().get().deepEquals(witness.getAstNode().get()) : "deepequals failed";
//    System.out.println("\nSuccessful deepequals");
  }

  public static void assertEquality(ViewSymbol witness, ViewSymbol expectedWitness) {
    List<String> cmpSymInnerExpectedList = new ArrayList<>();
    List<String> cmpSymSubExpectedList = new ArrayList<>();
    List<String> portSymExpectedList = new ArrayList<>();
    List<String> conSymExpectedList = new ArrayList<>();
    List<String> effSymExpectedList = new ArrayList<>();

    List<String> cmpSymInnerWitnessList = new ArrayList<>();
    List<String> cmpSymSubWitnessList = new ArrayList<>();
    List<String> portSymWitnessList = new ArrayList<>();
    List<String> conSymWitnessList = new ArrayList<>();
    List<String> effSymWitnessList = new ArrayList<>();

    //add topmost conns/effs
    expectedWitness.getConnectors().forEach(x -> conSymExpectedList.add(getConnEffString(x, null)));
    witness.getConnectors().forEach(x -> conSymWitnessList.add(getConnEffString(x, null)));
    expectedWitness.getEffectors().forEach(x -> effSymExpectedList.add(getConnEffString(x, null)));
    witness.getEffectors().forEach(x -> effSymWitnessList.add(getConnEffString(x, null)));

    //add inner elements
    fillLists(expectedWitness.getInnerComponents(), cmpSymInnerExpectedList, cmpSymSubExpectedList, portSymExpectedList, conSymExpectedList, effSymExpectedList);
    fillLists(witness.getInnerComponents(), cmpSymInnerWitnessList, cmpSymSubWitnessList, portSymWitnessList, conSymWitnessList, effSymWitnessList);

    if (witness.getName() != expectedWitness.getName()) {
      //replace the name of the witness by the name of the expected witness
      renameElements(witness.getName(), expectedWitness.getName(), cmpSymInnerWitnessList);
      renameElements(witness.getName(), expectedWitness.getName(), cmpSymSubWitnessList);
      renameElements(witness.getName(), expectedWitness.getName(), portSymWitnessList);
      renameElements(witness.getName(), expectedWitness.getName(), conSymWitnessList);
      renameElements(witness.getName(), expectedWitness.getName(), effSymWitnessList);
    }

    sortLists(cmpSymInnerExpectedList, cmpSymSubExpectedList, portSymExpectedList, conSymExpectedList, effSymExpectedList, cmpSymInnerWitnessList, cmpSymSubWitnessList, portSymWitnessList, conSymWitnessList, effSymWitnessList);

    assert cmpSymInnerExpectedList.equals(cmpSymInnerWitnessList) : "Inner components differ!";
    assert cmpSymSubExpectedList.equals(cmpSymSubWitnessList) : "Sub components differ!";
    assertEquals(portSymExpectedList, portSymWitnessList);
    assert conSymExpectedList.equals(conSymWitnessList) : "Connectors differ!";
    assert effSymExpectedList.equals(effSymWitnessList) : "Effectors differ!";

    System.out.println("Witnesses are equal!");
  }

  private static void fillLists(Collection<ViewComponentSymbol> csCollection, List<String> cmpSymInnerList, List<String> cmpSymSubList, List<String> portSymList, List<String> conSymList, List<String> effSymList) {
    for (ViewComponentSymbol cs : csCollection) {
      cmpSymInnerList.add(cs.getFullName());
      for (ViewPortSymbol p : cs.getPorts()) {
        portSymList.add((p.isIncoming() ? "in " : "out ") + p.getFullName() + ":" + p.getTypeName());
      }
      for (ViewConnectorSymbol conSym : cs.getConnectors()) {
        String connectorString = getConnEffString(conSym, cs.getFullName());
        conSymList.add(connectorString);
      }
      for (ViewEffectorSymbol effSym : cs.getEffectors()) {
        String effectorString = getConnEffString(effSym, cs.getFullName());
        effSymList.add(effectorString);
      }
      recurseSubComponents(cs.getSubComponents(), cmpSymSubList, portSymList, conSymList, effSymList);
    }
  }

  private static String getConnEffString(ViewConnectorSymbol conSym, String componentFullName) {
    return getConnEffInternal(componentFullName, conSym.getSourcePort(), conSym.getSource(), conSym.getTargetPort(), conSym.getTarget());
  }

  private static String getConnEffString(ViewEffectorSymbol effSym, String componentFullName) {
    return getConnEffInternal(componentFullName, effSym.getSourcePort(), effSym.getSource(), effSym.getTargetPort(), effSym.getTarget());
  }

  private static String getConnEffInternal(String componentFullName, ViewPortSymbol sourcePort, String symSource, ViewPortSymbol targetPort, String symTarget) {
    String start;
    if (sourcePort == null) {
      if (!componentFullName.equals(""))
        start = componentFullName + "." + symSource;
      else
        start = symSource;
    }
    else
      start = sourcePort.getFullName(); //this method seems to be buggy as it seems to use the capitalized instance name

    String end;
    if (targetPort == null) {
      if (!componentFullName.equals(""))
        end = componentFullName + "." + symTarget;
      else
        end = symTarget;
    }
    else
      end = targetPort.getFullName();
    return start + " -> " + end;
  }

  private static void recurseSubComponents(Collection<ViewComponentInstanceSymbol> cisCollection, List<String> cmpSymSubList, List<String> portSymList, List<String> conSymList, List<String> effSymList) {
    for (ViewComponentInstanceSymbol cis : cisCollection) {
      cmpSymSubList.add(cis.getFullName());
      for (ViewPortSymbol p : cis.getComponentType().getPorts()) {
        portSymList.add((p.isIncoming() ? "in " : "out ") + p.getFullName() + ":" + p.getTypeName());
      }
      for (ViewConnectorSymbol conSym : cis.getComponentType().getConnectors()) {
        String connectorString = getConnEffString(conSym, cis.getComponentType().getFullName());
        conSymList.add(connectorString);
      }
      for (ViewEffectorSymbol effSym : cis.getComponentType().getEffectors()) {
        String effectorString = getConnEffString(effSym, cis.getComponentType().getFullName());
        effSymList.add(effectorString);
      }
      recurseSubComponents(cis.getComponentType().getSubComponents(), cmpSymSubList, portSymList, conSymList, effSymList);
    }
  }

  private static void renameElements(String oldName, String newName, List<String> list) {
    List<String> newList = new ArrayList<>();
    list.forEach(x -> newList.add(x.replace(oldName, newName)));
    list.clear();
    list.addAll(newList);
  }

  private static void sortLists(List<String>... lists) {
    for (List<String> list : lists)
      Collections.sort(list);
  }

  public static void assertEquality(CnCView witness, CnCView expectedWitness) {
    List<String> cmpSymExpectedList = new ArrayList<>();
    List<String> portSymExpectedList = new ArrayList<>();
    List<String> conSymExpectedList = new ArrayList<>();
    List<String> effSymExpectedList = new ArrayList<>();
    List<String> effSymWitnessList = new ArrayList<>();
    List<String> conSymWitnessList = new ArrayList<>();
    List<String> portSymWitnessList = new ArrayList<>();
    List<String> cmpSymWitnessList = new ArrayList<>();

    cmpSymExpectedList.addAll(expectedWitness.getComponentNames());

  }
}
