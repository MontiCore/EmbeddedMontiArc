package de.monticore.lang.monticar.cnnarch.generator.configuration;

import de.monticore.generating.templateengine.GlobalExtensionManagement;

public class ConfLangGeneratorConfiguration {

    private static final GlobalExtensionManagement globalExtensionManagement = new GlobalExtensionManagement();

    public static GlobalExtensionManagement getGlobalExtensionManagement() {
        return globalExtensionManagement;
    }
}
