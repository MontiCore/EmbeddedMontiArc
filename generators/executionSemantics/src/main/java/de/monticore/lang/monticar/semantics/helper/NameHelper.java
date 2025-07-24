/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAElementInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortArraySymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;
import org.apache.commons.lang3.StringUtils;

import java.util.Optional;

public class NameHelper {

    public static String getPackageOfFullQualifiedName(String fullQualifiedName) {
        return StringUtils.substringBeforeLast(fullQualifiedName, ".");
    }

    public static String getName(String fullQualifiedName) {
        return StringUtils.substringAfterLast(fullQualifiedName, ".");
    }

    public static String toInstanceFullQualifiedName(String packageName, String componentName) {
        return Joiners.DOT.join(packageName, StringUtils.uncapitalize(componentName));
    }

    public static String toInstanceFullQualifiedName(String fullQualifiedName) {
        return Joiners.DOT.join(getPackageOfFullQualifiedName(fullQualifiedName), toInstanceName(fullQualifiedName));
    }

    public static String toInstanceName(String fullQualifiedName) {
        return StringUtils.uncapitalize(getName(fullQualifiedName));
    }

    public static String replaceWithDots(String str) {
        return str
                .replace("\\", ".")
                .replace("/", ".")
                .replace("_", ".");
    }

    public static String replaceWithUnderScore(String str) {
        return str
                .replace("\\", "_")
                .replace("/", "_")
                .replace(".", "_");
    }

    public static String calculateFullQualifiedNameOf(CommonSymbol symbol) {
        String fullName = null;
        if (!(symbol instanceof EMADynamicPortInstanceSymbol ||
                symbol instanceof EMADynamicConnectorInstanceSymbol ||
                symbol instanceof EMADynamicPortArraySymbol ||
                symbol instanceof EMADynamicComponentInstanceSymbol)) {
            try {
                fullName = symbol.getFullName();
            } catch (Exception e) {
            }
        }
        if (fullName != null && !fullName.equals(""))
            return fullName;
        return Names.getQualifiedName(symbol.getPackageName(), symbol.getName());
    }

    public static String calculatePartialName(EMAComponentInstanceSymbol childComponent,
                                              EMAComponentInstanceSymbol parentComponent) {
        String childName = calculatePartialName(calculateFullQualifiedNameOf(childComponent),
                calculateFullQualifiedNameOf(parentComponent));
        if (childName.equals(childComponent.getFullName())) {
            Optional<Symbol> child = parentComponent.getSpannedScope().resolveDown(childName, EMAComponentInstanceSymbol.KIND);
            if (child.isPresent()) {
                childName = childComponent.getName();
                Optional<EMAComponentInstanceSymbol> parent = childComponent.getParent();
                while (parent.isPresent() && !parent.get().equals(parentComponent)) {
                    childName = parent.get().getName() + childName;
                    parent = parent.get().getParent();
                }
            }
        }
        return childName;
    }

    public static String calculatePartialName(EMAPortInstanceSymbol childPort,
                                              EMAComponentInstanceSymbol parentComponent) {
        return Names.getQualifiedName(calculatePartialName(childPort.getComponentInstance(), parentComponent),
                childPort.getName());
    }

    public static String calculatePartialName(String childName, String parentName) {
        if (childName.equals(parentName)) return "";
        if (!childName.contains(parentName + ".")) return childName;
        return childName.substring(parentName.length() + 1);
    }
}
