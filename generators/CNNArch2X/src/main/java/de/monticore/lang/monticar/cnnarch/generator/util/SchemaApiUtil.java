package de.monticore.lang.monticar.cnnarch.generator.util;

import com.google.common.base.Joiner;

public class SchemaApiUtil {

    public static String createGetterMethodName(String... nameParts) {

        return "get_" + Joiner.on("_").join(nameParts);
    }
}
