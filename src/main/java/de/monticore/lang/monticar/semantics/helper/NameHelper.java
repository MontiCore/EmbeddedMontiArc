/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.helper;

import org.apache.commons.lang3.StringUtils;

public class NameHelper {

    public static String getPackageOfFullQualifiedName(String fullQualifiedName) {
        return StringUtils.substringBeforeLast(replaceWithDots(fullQualifiedName), ".");
    }

    public static String getName(String fullQualifiedName) {
        return StringUtils.substringAfterLast(replaceWithDots(fullQualifiedName), ".");
    }

    public static String toInstanceFullQualifiedName(String packageName, String componentName) {
        return replaceWithDots(packageName) + "." + StringUtils.uncapitalize(componentName);
    }

    public static String toInstanceFullQualifiedName(String fullQualifiedName) {
        return getPackageOfFullQualifiedName(fullQualifiedName) + "." + toInstanceName(fullQualifiedName);
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
}
