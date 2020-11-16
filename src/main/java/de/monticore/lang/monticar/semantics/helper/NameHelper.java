/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;
import org.apache.commons.lang3.StringUtils;

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
                .replace("\\",".")
                .replace("/",".")
                .replace("_", ".");
    }

    public static String replaceWithUnderScore(String str) {
        return str
                .replace("\\","_")
                .replace("/","_")
                .replace(".", "_");
    }

    public static String calculateFullQualifiedNameOf(EMAPortInstanceSymbol port) {
        return Names.getQualifiedName(port.getPackageName(), port.getName());
    }

    public static String calculateFullQualifiedNameOf(EMAComponentInstanceSymbol componentInstance) {
        return Names.getQualifiedName(componentInstance.getPackageName(), componentInstance.getName());
    }
}
