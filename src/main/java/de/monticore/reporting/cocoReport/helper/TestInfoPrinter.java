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

        infoErrored.put("Number", "" + (info.invalid + Integer.parseInt((String) infoErrored.get("Number"))));
        infoErrored.put("Valid", "" + 0);
        infoErrored.put("Invalid", "" + (info.invalid + Integer.parseInt((String) infoErrored.get("Invalid"))));

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
            String str = null;
            try {
                str = FileUtils.readFileToString(infoFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
            JSONParser jsonParser = new JSONParser();
            try {
                return (JSONObject) jsonParser.parse(str);
            } catch (ParseException e) {
                e.printStackTrace();
            }
            return null;
        }
    }

    public static void printInfo_(List<CheckCoCoResult> testResults, String infoPath, Date date, boolean merge){
        if (testResults.size() == 0) return;
        if (merge) {
            try {
                String first = FileUtils.readFileToString(new File(infoPath));
                first = first.substring(0, first.length() - 3);
                String str = first + ",\n" + getInfo(testResults, date, merge);
                FileUtils.writeStringToFile(new File(infoPath),
                        str);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            try {
                FileUtils.writeStringToFile(new File(infoPath),
                        getInfo(testResults, date, merge));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private static String getInfo(List<CheckCoCoResult> testResults, Date date,  boolean merge){
        ValidInfo info = getValidInfo(testResults);

        IndentPrinter ip = new IndentPrinter();
        if (!merge)
            ip.println("{");
        ip.indent();

        ip.println("\"" + info.root + "\": {");
        ip.indent();
        ip.println("\"Number\": \"" + info.number + "\",");
        ip.println("\"Valid\": \"" + info.valid + "\",");
        ip.println("\"Invalid\": \"" + info.invalid + "\"");
        ip.unindent();

        ip.print("},");
        ip.println();

        ip.println("\"" + CheckCoCoResult.erroredString + "_" + info.root + "\": {");
        ip.indent();
        ip.println("\"Number\": \"" + info.errored + "\",");
        ip.println("\"Valid\": \"" + 0 + "\",");
        ip.println("\"Invalid\": \"" + info.errored + "\"");
        ip.unindent();
        ip.print("},");
        ip.println();

        String formattedTimeStamp = (new SimpleDateFormat("dd.MM.yyyy HH:mm").format(date));
        ip.print("\"date\": " + "\"" + formattedTimeStamp + "\"");
        ip.println();

        ip.unindent();
        ip.println("}");

        return ip.getContent();
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
                fallBackRoot = testResult.getRootName();
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
