/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.witness;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewSymbol;
import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.EmbeddedMontiViewLoader;
import de.rwth.cnc.viewverification.VerificationHelper;
import de.rwth.cnc.viewverification.inconsistency.*;

import java.io.StringReader;
import java.nio.file.Path;
import java.util.Scanner;
import java.util.Stack;

public class WitnessGenerator {

  public static CnCView generateWitnessesForInconsistencyData(InconsistenciesData id) {
    CnCArchitecture model = id.getModel();
    CnCView view = id.getView();
    int fileId = 0;

    CnCView witness = null;

    for (InconsistencyMissingComponent missingCmp : id.getMissingComponents()) {
      witness = GenerateInconsistencyView.getViewForMissingComponent(model, view, missingCmp, Integer.toString(fileId));
      String text = getViewText(witness);
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "missingComponent", "MissingComponent" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      missingCmp.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyInterfaceMismatch im : id.getInterfaceMismatches()) {
      witness = GenerateInconsistencyView.getViewForInterfaceMismatch(model, view, im, Integer.toString(fileId));
      String text = getViewText(witness);
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "", "InterfaceMismatch" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      im.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyHierarchyMismatch hm : id.getHierarchyMismatches()) {
      witness = GenerateInconsistencyView.getViewForHierarchyMismatch(model, view, hm, Integer.toString(fileId));
      String text = getViewText(witness);
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "", "HierarchyMismatch" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      hm.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyMissingConnection mc : id.getMissingConnections()) {
      witness = GenerateInconsistencyView.getViewForMissingConnection(model, view, mc, Integer.toString(fileId));
      String text = getViewText(witness);
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "", "MissingConnection" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      mc.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyMissingEffector me : id.getMissingEffectors()) {
      witness = GenerateInconsistencyView.getViewForMissingEffector(model, view, me, Integer.toString(fileId));
      String text = getViewText(witness);
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "", "MissingEffector" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      me.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyNotAtomic na : id.getNotAtomicMismatches()) {
      witness = GenerateInconsistencyView.getViewForNotAtomicMismatch(model, view, na, Integer.toString(fileId));
      String text = getViewText(witness);
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "", "NotAtomic" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      na.setJustificationFileName(filePath.toAbsolutePath().toString());
    }

    fileId = 0;
    for (InconsistencyIFCViolation iv : id.getIFCViolations()) {
      witness = GenerateInconsistencyView.getViewForIFCViolation(model, view, iv, Integer.toString(fileId));
      String text = getViewText(witness);
      Path filePath = WitnessGeneratorHelper.getWitnessFilePath(false, view, "", "IFCViolation" + Integer.toString(fileId));
      fileId++;
      WitnessGeneratorHelper.writeStringToFile(filePath, text);

      iv.setJustificationFileName(filePath.toAbsolutePath().toString());
    }
    return witness;
  }

  public static CnCView generateWitnessForConsistency(CnCArchitecture model, CnCView view) {
    CnCView witness = GeneratePositiveWitnessView.getViewForPositiveWitness(model, view);
    String text = getViewText(witness);
    Path filePath = WitnessGeneratorHelper.getPositiveWitnessFilePath(model, view);
    WitnessGeneratorHelper.writeStringToFile(filePath, text);
    return witness;
  }

  private static String getViewText(CnCView view) {
    //ComponentSymbol component = EmbeddedMontiViewLoader.convertToEMVComponent(view);
    //String viewText = SymbolPrinter.printComponent(component);
    ViewSymbol viewSym = EmbeddedMontiViewLoader.convertToEMVView(view);
    String viewText = view.getComment() + "\n\n" + SymbolPrinter.printView(viewSym);

    //fix missing instances:
    viewText = fixMissingInstances(viewText);

    return viewText;
  }

  // adds instance C c; after every } that is not the view or top component
  // does not ignore multiline or inline comments! /* */
  private static String fixMissingInstances(String viewText) {

    StringBuilder builder = new StringBuilder();
    Scanner scanner = new Scanner(viewText);

    int currentNesting = 0;
    Stack<String> currentComponentStack = new Stack<>();
    while (scanner.hasNextLine()) {
      String line = scanner.nextLine();

      builder.append(line).append("\n");
      if (!line.trim().startsWith("//")) {
        if (line.contains(" component ")) {
          String[] lineSplit = line.trim().split(" ");
          //assert lineSplit.length < 4 : "badly formatted line: " + line;
          //assert lineSplit.length > 2 : "badly formatted line: " + line;
          if (lineSplit.length > 4 || lineSplit.length < 2) {
          }
          else
            currentComponentStack.push(lineSplit[1]);
        }

        int open = (int) line.chars().filter(ch -> ch == '{').count();
        int close = (int) line.chars().filter(ch -> ch == '}').count();
        assert close < 2 : "badly formatted line: " + line;
        currentNesting += open;
        currentNesting -= close;
        if (close > 0 && currentNesting >= 2) {
          String leadingWhiteSpace = line;
          // trim trailing w/s
          leadingWhiteSpace = leadingWhiteSpace.replaceAll("\\s+$", "");
          // remove all encloses w/s
          leadingWhiteSpace = leadingWhiteSpace.replaceAll("[^\\s]\\s[^\\s]", "");
          // remove all non w/s characters
          leadingWhiteSpace = leadingWhiteSpace.replaceAll("[^\\s]", "");
          assert leadingWhiteSpace.chars().allMatch(Character::isWhitespace) : "Contains whitespace: " + leadingWhiteSpace;

          //instance C c;\n
          String currentComponent = currentComponentStack.pop();
          builder.append(leadingWhiteSpace + "instance ").append(currentComponent).append(" ").append(VerificationHelper.uncapitalize(currentComponent)).append(";").append(("\n"));
        }
      }
    }
    return builder.toString();
  }

}
