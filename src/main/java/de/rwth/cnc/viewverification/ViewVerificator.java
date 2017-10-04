package de.rwth.cnc.viewverification;

import java.util.LinkedList;
import java.util.List;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.checks.*;
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
        return verify(modelPath, modelName, viewPath, viewName, true);
    }

    public static List<InconsistencyItem> verify(String modelPath, String modelName, String viewPath, String viewName, boolean generateWitnesses) {
        CnCArchitecture model = loadComponent(modelPath, modelName);
        CnCView view = loadView(viewPath, viewName);
        return verify(model, view, generateWitnesses);
    }

    public static List<InconsistencyItem> verify(CnCArchitecture model, CnCView view) {
        return verify(model, view, true);
    }

    public static List<InconsistencyItem> verify(CnCArchitecture model, CnCView view, boolean generateWitnesses) {
        InconsistenciesData id = runChecks(model, view);

        if (generateWitnesses) {
            if (id.hasEntries()) {
                WitnessGenerator.generateWitnessesForInconsistencyData(id);
            } else {
                WitnessGenerator.generateWitnessForConsistency(model, view);
            }
        }

        //Split all inconsistencies into single objects
        List<InconsistencyItem> inconsistencyItems = buildInconsistencyItems(id);
        return inconsistencyItems;
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