package de.monticore.lang.monticar.generator.roscpp.helper;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.IOUtils;

import java.util.HashMap;
import java.util.Map;

public class TemplateHelper {
    private static Map<String, String> cache = new HashMap<>();

    private static String getTemplate(String fileName) {
        if (cache.containsKey(fileName)) {
            return cache.get(fileName);
        }

        String tmpStr = "";
        String resourceFileName = "/de/monticore/lang/monticar/generator/roscpp/" + fileName;

        try {
            tmpStr = IOUtils.toString(TemplateHelper.class.getResourceAsStream(resourceFileName));
        } catch (Exception e) {
            //Not recoverable
            Log.error("Template file not found: " + resourceFileName);
        }

        cache.put(fileName, tmpStr);
        return tmpStr;
    }

    public static String getCMakeListsTemplate() {
        return getTemplate("CMakeLists.template");
    }

    public static String getMsgGenTemplate() {
        return getTemplate("msgGen.template");
    }

    public static String getRos2MsgGenTemplate() {
        return getTemplate("ros2MsgGen.template");
    }

    public static String getGenerateMsgsPyTemplate() {
        return getTemplate("generateMsgs.py.template");
    }

}
