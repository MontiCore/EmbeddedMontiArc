package de.monticore.lang.monticar.generator.middleware.helpers;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class TemplateHelper {
    private static Map<String, String> cache = new HashMap<>();

    private static String getTemplate(String fileName) {
        if (cache.containsKey(fileName)) {
            return cache.get(fileName);
        }

        String tmpStr = "";
        String fullFileName = "src/main/resources/de/monticore/lang/monticar/generator/middleware/" + fileName;
        try {
            tmpStr = FileUtils.readFileToString(new File(fullFileName));
        } catch (Exception e) {
            //Not recoverable
            Log.error("Template file not found: " + fullFileName);
        }

        cache.put(fileName, tmpStr);
        return tmpStr;
    }

    public static String getCoordinatorTemplate() {
        return getTemplate("coordinatorTemplate.ftl");
    }

    public static String getStruct_msgsCmakeTemplate() {
        return getTemplate("struct_msgsCmakeTemplate.ftl");
    }

    public static String getDummyCmakeTemplate() {
        return getTemplate("dummyCmakeTemplate.ftl");
    }

    public static String getDummyAdapterTemplate() {
        return getTemplate("dummyAdapterTemplate.ftl");
    }

    public static String getIAdapterTemplate() {
        return getTemplate("IAdapterTemplate.ftl");
    }

    public static String getCoordinatorCmakeListsTemplate() {
        return getTemplate("coordinatorCmakeListsTemplate.ftl");
    }

    public static String getCmakeCppTemplate() {
        return getTemplate("cmakeCppTemplate.ftl");
    }
}
