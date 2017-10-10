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
package de.rwth.cnc.viewverification;

import java.lang.reflect.Type;
import java.util.*;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.checks.*;
import de.rwth.cnc.viewverification.helper.StringIntTuple;
import de.rwth.cnc.viewverification.helper.TypeVerificator;
import de.rwth.cnc.viewverification.witness.*;
import de.rwth.cnc.viewverification.inconsistency.*;

public class ViewVerificator {

  private static CnCArchitecture loadComponent(String modelPath, String modelName) {
    System.out.println("START LOADING COMP");
    CnCArchitecture cncSys = EmbeddedMontiArcLoader.loadComponent(modelPath, modelName);
    System.out.println("DONE LOADING COMP");
    return cncSys;
  }

  private static CnCView loadView(String viewPath, String viewName) {
    System.out.println("START LOADING VIEW");
    CnCView cncView = EmbeddedMontiViewLoader.loadView(viewPath, viewName);
    System.out.println("DONE LOADING VIEW");
    return cncView;
  }

  public static List<InconsistencyItem> verify(String modelPath, String modelName, String viewPath, String viewName) {
    return verify(modelPath, modelName, viewPath, viewName, true, true);
  }

  public static List<InconsistencyItem> verify(String modelPath, String modelName, String viewPath, String viewName, boolean generateWitnesses, boolean ignoreInstanceNames) {
    CnCArchitecture model = loadComponent(modelPath, modelName);
    CnCView view = loadView(viewPath, viewName);
    return verify(model, view, generateWitnesses, ignoreInstanceNames);
  }

  public static List<InconsistencyItem> verify(CnCArchitecture model, CnCView view) {
    return verify(model, view, true);
  }

  public static List<InconsistencyItem> verify(CnCArchitecture model, CnCView view, boolean generateWitnesses) {
    InconsistenciesData id = runChecks(model, view);

    if (generateWitnesses) {
      if (id.hasEntries()) {
        WitnessGenerator.generateWitnessesForInconsistencyData(id);
      }
      else {
        WitnessGenerator.generateWitnessForConsistency(model, view);
      }
    }

    //Split all inconsistencies into single objects
    List<InconsistencyItem> inconsistencyItems = buildInconsistencyItems(id);
    return inconsistencyItems;
  }

  public static List<InconsistencyItem> verify(CnCArchitecture model, CnCView view, boolean generateWitnesses, boolean ignoreInstanceNames) {
    if (!ignoreInstanceNames)
      return verify(model, view, generateWitnesses);

    boolean cmpTypesFitting = TypeVerificator.checkComponentTypes(model, view);

    if (cmpTypesFitting) {

      //Bruteforce eligible variation:

      List<CnCView> bfViewList = TypeVerificator.bruteforceRenamedViews(model, view);

      for (CnCView v : bfViewList) {
        List<InconsistencyItem> result = verify(model, v, false);
        if (result.size() == 0)
          if (generateWitnesses) {
            return verify(model, v, true);
          }
          else {
            return result;
          }
      }
    }
    else {
      //No eligible variation possible.
      InconsistencyItem ii = new InconsistencyItem(InconsistencyKind.TYPEVERIFCATIONERROR, "", "No eligible map from the view to the model possible. Probably missing component type or typo.");
      return Arrays.asList(ii);
    }

    //No eligible variation found.
    InconsistencyItem ii = new InconsistencyItem(InconsistencyKind.TYPEVERIFCATIONERROR, "", "No eligible map from the view to the model could be found.");
    return Arrays.asList(ii);
  }

  private static InconsistenciesData runChecks(CnCArchitecture model, CnCView view) {
    InconsistenciesData id = new InconsistenciesData();

    id.setModel(model);
    id.setView(view);

    CheckExistenceOfComponents mcc = new CheckExistenceOfComponents(model, view);
    if (!mcc.checkConsistency()) {
      for (String cmpName : mcc.getMissingComponents()) {
        id.addMissingComponent(new InconsistencyMissingComponent(cmpName));
      }
    }

    CheckInterfaces ci = new CheckInterfaces(model, view);
    if (!ci.checkConsistency()) {
      for (InconsistencyInterfaceMismatch iim : ci.getInterfaceMismatches()) {
        id.addInterfaceMismatch(iim);
      }
    }

    CheckHierarchy ch = new CheckHierarchy(model, view);
    if (!ch.checkConsistency()) {
      for (InconsistencyHierarchyMismatch hm : ch.getHierarchyMismatches()) {
        id.addHierarchyMismatch(hm);
      }
    }

    CheckConnectors cc = new CheckConnectors(model, view);
    if (!cc.checkConsistency()) {
      for (InconsistencyMissingConnection imc : cc.getMissingConnections()) {
        id.addMissingConnection(imc);
      }
    }

    CheckEffectors ce = new CheckEffectors(model, view);
    if (!ce.checkConsistency()) {
      for (InconsistencyMissingEffector imc : ce.getMissingEffectors()) {
        id.addMissingEffector(imc);
      }
    }

    CheckAtomic checkAtomic = new CheckAtomic(model, view);
    if (!checkAtomic.checkConsistency()) {
      for (InconsistencyNotAtomic ina : checkAtomic.getNotAtomicMismatches()) {
        id.addNotAtomicMismatch(ina);
      }
    }

    CheckIFC checkIFC = new CheckIFC(model, view);
    if (!checkIFC.checkConsistency()) {
      for (InconsistencyIFCViolation iifc : checkIFC.getIfcViolations()) {
        id.addIFCViolation(iifc);
      }
    }

    return id;
  }

  private static List<InconsistencyItem> buildInconsistencyItems(InconsistenciesData id) {
    List<InconsistencyItem> inconsistencyItems = new LinkedList<>();
    String archName = id.getModel().getName();
    String viewName = id.getView().getName();

    InconsistencyItem in;
    for (InconsistencyMissingComponent missingCmp : id.getMissingComponents()) {
      in = new InconsistencyItem(InconsistencyKind.MISSING_CMP, missingCmp.getJustificationFileName(), GenerateInconsistencyDesc.getDescForMissingComponent(archName, viewName, missingCmp));
      inconsistencyItems.add(in);
    }

    for (InconsistencyHierarchyMismatch ihm : id.getHierarchyMismatches()) {
      in = new InconsistencyItem(InconsistencyKind.HIERARCHY_MISMATCH, ihm.getJustificationFileName(), GenerateInconsistencyDesc.getDescForHierarchyMismatch(archName, viewName, ihm));
      inconsistencyItems.add(in);
    }

    for (InconsistencyInterfaceMismatch iim : id.getInterfaceMismatches()) {
      in = new InconsistencyItem(InconsistencyKind.INTERFACE_MISMATCH, iim.getJustificationFileName(), GenerateInconsistencyDesc.getDescForInterfaceMismatch(archName, viewName, iim));
      inconsistencyItems.add(in);
    }

    for (InconsistencyMissingConnection imc : id.getMissingConnections()) {
      in = new InconsistencyItem(InconsistencyKind.MISSING_CON, imc.getJustificationFileName(), GenerateInconsistencyDesc.getDescForMissingConnection(archName, viewName, imc));
      inconsistencyItems.add(in);
    }

    for (InconsistencyMissingEffector imc : id.getMissingEffectors()) {
      in = new InconsistencyItem(InconsistencyKind.MISSING_EFF, imc.getJustificationFileName(), GenerateInconsistencyDesc.getDescForMissingEffector(archName, viewName, imc));
      inconsistencyItems.add(in);
    }

    for (InconsistencyNotAtomic ina : id.getNotAtomicMismatches()) {
      in = new InconsistencyItem(InconsistencyKind.NOT_ATOMIC, ina.getJustificationFileName(), GenerateInconsistencyDesc.getDescForNotAtomicMismatch(archName, viewName, ina));
      inconsistencyItems.add(in);
    }

    for (InconsistencyIFCViolation iifc : id.getIFCViolations()) {
      in = new InconsistencyItem(InconsistencyKind.IFC_VIOLATION, iifc.getJustificationFileName(), GenerateInconsistencyDesc.getDescForIFCViolation(archName, viewName, iifc));
      inconsistencyItems.add(in);
    }

    return inconsistencyItems;
  }

}