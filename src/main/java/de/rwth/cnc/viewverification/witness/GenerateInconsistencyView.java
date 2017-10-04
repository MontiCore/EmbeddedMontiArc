package de.rwth.cnc.viewverification.witness;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.inconsistency.*;

public class GenerateInconsistencyView {

    public static CnCView getViewForMissingComponent(CnCArchitecture system, CnCView view, InconsistencyMissingComponent cmpName) {

        CnCView witnessView = new CnCView();

        witnessView.setName("MissingComponent");
        witnessView.setComment("// " + GenerateInconsistencyDesc.getDescForMissingComponent(system.getName(), view.getName(), cmpName));

        return witnessView;
    }

    public static CnCView getViewForHierarchyMismatch(CnCArchitecture system, CnCView view, InconsistencyHierarchyMismatch hierarchyMismatch) {

        CnCView witnessView = new CnCView();
        witnessView.setName("HierarchyMismatch");
        witnessView.setComment("// " + GenerateInconsistencyDesc.getDescForHierarchyMismatch(system.getName(), view.getName(), hierarchyMismatch));

        String cmp1 = hierarchyMismatch.getComponentChild();
        String cmp2 = hierarchyMismatch.getComponentParent();
        String lcParent = system.getLeastCommonParent(cmp1, cmp2).getName();

        List<Component> p1 = WitnessGeneratorHelper.getArchPath(system, lcParent, cmp1);
        p1.remove(0);
        List<Component> p2 = WitnessGeneratorHelper.getArchPath(system, lcParent, cmp2);
        p2.remove(0);
        Component leastCommonParent = new Component();
        leastCommonParent.setName(lcParent);
        if (p1.size() > 0) {
            leastCommonParent.addContainedComponent(p1.get(0).getName());
        }
        if (p2.size() > 0) {
            leastCommonParent.addContainedComponent(p2.get(0).getName());
        }
        witnessView.addComponent(leastCommonParent);
        witnessView.getComponents().addAll(p1);
        witnessView.getComponents().addAll(p2);
        List<String> topLevelCmp = new ArrayList<String>();
        topLevelCmp.add(lcParent);
        witnessView.setTopLevelComponentNames(topLevelCmp);

        return witnessView;
    }

    public static CnCView getViewForInterfaceMismatch(CnCArchitecture system, CnCView view, InconsistencyInterfaceMismatch interfaceMismatch) {

        CnCView witnessView = new CnCView();

        witnessView.setName("InterfaceMismatch");
        witnessView.setComment("// " + GenerateInconsistencyDesc.getDescForInterfaceMismatch(system.getName(), view.getName(), interfaceMismatch));

        Component sysCmp = system.getComponent(interfaceMismatch.getComponentName());
        Component viewCmp = new Component();
        viewCmp.setName(sysCmp.getName());

        if (interfaceMismatch.getMismatchKind().equals(InconsistencyInterfaceMismatchKind.NO_MATCH)) {
            for (Port port : sysCmp.getPorts()) {
                viewCmp.addPort(port.clone());
            }
        } else {
            Port p = sysCmp.getPort(interfaceMismatch.getPortName());
            viewCmp.addPort(p.clone());
        }

        witnessView.addComponent(viewCmp);
        witnessView.addTopLevelComponentName(viewCmp.getName());

        return witnessView;
    }

    public static CnCView getViewForMissingConnection(CnCArchitecture arch, CnCView view, InconsistencyMissingConnection missingConnection) {

        CnCView witnessView = new CnCView();

        witnessView.setName("MissingConnection");
        witnessView.setComment("// " + GenerateInconsistencyDesc.getDescForMissingConnection(arch.getName(), view.getName(), missingConnection));

        String srcCmpName = missingConnection.getComponentSource();
        String tgtCmpName = missingConnection.getComponentTarget();
        // compute reachable components (to determine top level component)
        Set<String> cmpsInView = arch.getReachableComponents(srcCmpName, false);
        cmpsInView.add(srcCmpName);
        cmpsInView.add(tgtCmpName);

        String parentCmpName = arch.getLeastCommonParent(cmpsInView).getName();

        List<Component> p1 = arch.getArchPath(parentCmpName, srcCmpName);
        p1.remove(0);
        List<Component> p2 = arch.getArchPath(parentCmpName, tgtCmpName);
        p2.remove(0);
        Component parentCmp = new Component();
        parentCmp.setName(parentCmpName);
        if (p1.size() > 0) {
            parentCmp.addContainedComponent(p1.get(0).getName());
        }
        if (p2.size() > 0) {
            parentCmp.addContainedComponent(p2.get(0).getName());
        }
        witnessView.addComponent(parentCmp);
        witnessView.getComponents().addAll(p1);
        witnessView.getComponents().addAll(p2);
        List<String> topLevelCmp = new ArrayList<String>();
        topLevelCmp.add(parentCmpName);
        witnessView.setTopLevelComponentNames(topLevelCmp);

        for (Port p : arch.getComponent(srcCmpName).getPorts()) {
            witnessView.getComponent(srcCmpName).addPort(p.clone());
            WitnessGeneratorHelper.addConnectorTargets(srcCmpName + "." + p.getName(), arch, witnessView);
        }

        return witnessView;
    }

    /**
     * FIXME just a copy of missing connectors for now
     *
     * @param arch
     * @param view
     * @param missingConnection
     * @return
     */
    public static CnCView getViewForMissingEffector(CnCArchitecture arch, CnCView view, InconsistencyMissingEffector missingConnection) {

        CnCView witnessView = new CnCView();

        witnessView.setName("MissingEffector");
        witnessView.setComment("// " + GenerateInconsistencyDesc.getDescForMissingEffector(arch.getName(), view.getName(), missingConnection));

        String srcCmpName = missingConnection.getComponentSource();
        String tgtCmpName = missingConnection.getComponentTarget();
        // compute reachable components (to determine top level component)
        Set<String> cmpsInView = arch.getReachableComponents(srcCmpName, true);
        cmpsInView.add(srcCmpName);
        cmpsInView.add(tgtCmpName);

        String parentCmpName = arch.getLeastCommonParent(cmpsInView).getName();

        List<Component> p1 = arch.getArchPath(parentCmpName, srcCmpName);
        p1.remove(0);
        List<Component> p2 = arch.getArchPath(parentCmpName, tgtCmpName);
        p2.remove(0);
        Component parentCmp = new Component();
        parentCmp.setName(parentCmpName);
        if (p1.size() > 0) {
            parentCmp.addContainedComponent(p1.get(0).getName());
        }
        if (p2.size() > 0) {
            parentCmp.addContainedComponent(p2.get(0).getName());
        }
        witnessView.addComponent(parentCmp);
        witnessView.getComponents().addAll(p1);
        witnessView.getComponents().addAll(p2);
        List<String> topLevelCmp = new ArrayList<String>();
        topLevelCmp.add(parentCmpName);
        witnessView.setTopLevelComponentNames(topLevelCmp);

        for (Port p : arch.getComponent(srcCmpName).getPorts()) {
            witnessView.getComponent(srcCmpName).addPort(p.clone());
            WitnessGeneratorHelper.addConnectorTargets(srcCmpName + "." + p.getName(), arch, witnessView);
        }

        return witnessView;
    }

    public static CnCView getViewForNotAtomicMismatch(CnCArchitecture arch, CnCView view, InconsistencyNotAtomic inconsistency) {
        CnCView witnessView = new CnCView();
        witnessView.setName("NotAtomic");
        witnessView.setComment("// " + GenerateInconsistencyDesc.getDescForNotAtomicMismatch(arch.getName(), view.getName(), inconsistency));

        String component = inconsistency.getComponentName();
        Component cmp = new Component();
        cmp.setName(component);
        witnessView.addComponent(cmp);

        List<String> subcomponents = arch.getComponent(component).getContainedComponents();

        List<String> topLevelComponents = new ArrayList<>();
        topLevelComponents.add(component);
        witnessView.setTopLevelComponentNames(topLevelComponents);
        for (String subComp : subcomponents) {
            Component c = new Component();
            c.setName(subComp);

            cmp.addContainedComponent(subComp);
            witnessView.addComponent(c);
        }

        return witnessView;
    }

    public  static CnCView getViewForIFCViolation(CnCArchitecture arch, CnCView view, InconsistencyIFCViolation inconsistency)
    {
        CnCView witnessView = new CnCView();
        witnessView.setName("Interface-CompletenessViolation");
        witnessView.setComment("// " + GenerateInconsistencyDesc.getDescForIFCViolation(arch.getName(), view.getName(), inconsistency));

        String component = inconsistency.getComponentName();
        Component cmp = new Component();
        cmp.setName(component);

        Port p = new Port();
        p.setName(inconsistency.getPortName());
        p.setType(inconsistency.getPortType());
        p.setDirection(inconsistency.getPortDirection());
        p.setComponent(cmp);
        cmp.addPort(p);

        witnessView.addComponent(cmp);

        return witnessView;
    }

}
