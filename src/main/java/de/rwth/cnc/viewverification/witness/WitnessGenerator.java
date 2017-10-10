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
package de.rwth.cnc.viewverification.witness;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewSymbol;
import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.EmbeddedMontiViewLoader;
import de.rwth.cnc.viewverification.inconsistency.*;

import java.nio.file.Path;

public class WitnessGenerator {

  public static void generateWitnessesForInconsistencyData(InconsistenciesData id) {
    CnCArchitecture model = id.getModel();
    CnCView view = id.getView();
    int fileId = 0;

    for (InconsistencyMissingComponent missingCmp : id.getMissingComponents()) {
      String text = getViewText(GenerateInconsistencyView.getViewForMissingComponent(model, view, missingCmp));
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "missingComponent", "missingComponent" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      missingCmp.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyInterfaceMismatch im : id.getInterfaceMismatches()) {
      String text = getViewText(GenerateInconsistencyView.getViewForInterfaceMismatch(model, view, im));
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "interfaceMismatch", "interfaceMismatch" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      im.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyHierarchyMismatch hm : id.getHierarchyMismatches()) {
      String text = getViewText(GenerateInconsistencyView.getViewForHierarchyMismatch(model, view, hm));
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "hierarchyMismatch", "hierarchyMismatch" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      hm.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyMissingConnection mc : id.getMissingConnections()) {
      String text = getViewText(GenerateInconsistencyView.getViewForMissingConnection(model, view, mc));
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "missingConnection", "missingConnection" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      mc.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyMissingEffector me : id.getMissingEffectors()) {
      String text = getViewText(GenerateInconsistencyView.getViewForMissingEffector(model, view, me));
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "missingEffector", "missingEffector" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      me.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyNotAtomic na : id.getNotAtomicMismatches()) {
      String text = getViewText(GenerateInconsistencyView.getViewForNotAtomicMismatch(model, view, na));
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "notAtomic", "notAtomic" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      na.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyIFCViolation iv : id.getIFCViolations()) {
      String text = getViewText(GenerateInconsistencyView.getViewForIFCViolation(model, view, iv));
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "ifcViolation", "ifcViolation" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      iv.setJustificationFileName(filePath.toAbsolutePath().toString());
    }
  }

  public static void generateWitnessForConsistency(CnCArchitecture model, CnCView view) {
    String text = getViewText(GeneratePositiveWitnessView.getViewForPositiveWitness(model, view));
    Path filePath = WitnessGeneratorHelper.getWitnessFilePath(true, view, "", model.getName());
    WitnessGeneratorHelper.writeStringToFile(filePath, text);
  }

  private static String getViewText(CnCView view) {
    //ComponentSymbol component = EmbeddedMontiViewLoader.convertToEMVComponent(view);
    //String viewText = SymbolPrinter.printComponent(component);
    ViewSymbol viewSym = EmbeddedMontiViewLoader.convertToEMVView(view);
    String viewText = view.getComment() + "\n\n" + SymbolPrinter.printView(viewSym);
    return viewText;
  }

}
