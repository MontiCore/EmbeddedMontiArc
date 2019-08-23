/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.reporting.helper.OrderableModelInfo;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

public class TestInfoPrinter {

    public static void printInfo(List<CheckCoCoResult> testResults, String infoPath, Date date, boolean merge){
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

        String tempRoot = "";


        for(CheckCoCoResult testResult: testResults){
            number++;
            if(root.equals("") && !testResult.getRootName1().contains(OrderableModelInfo.erroredString))
                root = testResult.getRootName1();
            else if (root.equals(""))
                tempRoot = testResult.getRootName1();
            if(testResult.isValid())
                valid++;
            else
                invalid++;
            if(testResult.getParsed() != 1 || testResult.getResolved() != 1)
                errored++;
        }
        if ( root.equals("")) root = tempRoot;
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
