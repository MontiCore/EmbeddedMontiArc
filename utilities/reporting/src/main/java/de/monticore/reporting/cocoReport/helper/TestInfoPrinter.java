/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.helper.OrderableModelInfo;
import org.apache.commons.io.FileUtils;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

public class TestInfoPrinter {

    public static void printInfo(List<CheckCoCoResult> testResults, String infoPath, Date date){
        File infoFile = new File(infoPath);
        boolean merge = false;
        if (infoFile.exists()) merge = true;

        ValidInfo info = getValidInfo(testResults);
        JSONObject infoJSON = getJSONFile(infoFile, merge, info, date);

        JSONObject infoValid = (JSONObject) infoJSON.get(info.root);
        JSONObject infoErrored = (JSONObject) infoJSON.get(CheckCoCoResult.erroredString + "_" + info.root);

        infoValid.put("Number", "" + (info.number + Integer.parseInt((String) infoValid.get("Number"))));
        infoValid.put("Valid", "" + (info.valid + Integer.parseInt((String) infoValid.get("Valid"))));
        infoValid.put("Invalid", "" + (info.invalid + Integer.parseInt((String) infoValid.get("Invalid"))));

        infoErrored.put("Number", "" + (info.errored + Integer.parseInt((String) infoErrored.get("Number"))));
        infoErrored.put("Valid", "" + 0);
        infoErrored.put("Invalid", "" + (info.errored + Integer.parseInt((String) infoErrored.get("Invalid"))));

        try {
            FileUtils.write(infoFile, infoJSON.toJSONString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static JSONObject getJSONFile(File infoFile, boolean merge, ValidInfo info, Date date) {
        if(!merge) {
            JSONObject infoJSON = new JSONObject();
            JSONObject infoValid = new JSONObject();
            JSONObject infoErrored = new JSONObject();

            infoJSON.put(info.root, infoValid);
            infoJSON.put(CheckCoCoResult.erroredString + "_" + info.root, infoErrored);

            infoValid.put("Number", "" + 0);
            infoValid.put("Valid", "" + 0);
            infoValid.put("Invalid", "" + 0);

            infoErrored.put("Number", "" + 0);
            infoErrored.put("Valid", "" + 0);
            infoErrored.put("Invalid", "" + 0);

            String formattedTimeStamp = (new SimpleDateFormat("dd.MM.yyyy HH:mm").format(date));
            infoJSON.put("date", formattedTimeStamp);

            return infoJSON;
        } else {
            JSONParser jsonParser = new JSONParser();
            try {
                return (JSONObject) jsonParser.parse(FileUtils.readFileToString(infoFile));
            } catch (ParseException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
            return null;
        }
    }

    private static ValidInfo getValidInfo(List<CheckCoCoResult> testResults){
        TestInfoPrinter tif = new TestInfoPrinter();
        return tif.getValidInfo_(testResults);
    }

    private ValidInfo getValidInfo_(List<CheckCoCoResult> testResults){
        ValidInfo info = new ValidInfo();
        String root = "";
        int number = 0;
        int valid = 0;
        int invalid = 0;
        int errored = 0;

        String fallBackRoot = "";
        for(CheckCoCoResult testResult: testResults){
            number++;
            if(root.equals("") && !testResult.getRootName().contains(OrderableModelInfo.erroredString))
                root = testResult.getRootName();
            else if (root.equals(""))
                fallBackRoot = testResult.getRootName().substring(OrderableModelInfo.erroredString.length() + 1);
            if(testResult.isValid())
                valid++;
            else
                invalid++;
            if(testResult.getParsed() != 1 || testResult.getResolved() != 1)
                errored++;
        }
        if (root.equals("")) root = fallBackRoot;
        info.root = root;
        info.number = number;
        info.valid = valid;
        info.invalid = invalid;
        info.errored = errored;

        return info;
    }

    public class ValidInfo {
        public String root;
        public int number;
        public int valid;
        public int invalid;
        public int errored;
    }
}
