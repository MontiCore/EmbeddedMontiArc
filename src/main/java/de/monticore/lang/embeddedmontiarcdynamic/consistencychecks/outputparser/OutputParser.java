package de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.outputparser;

import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.enums.Enums;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class OutputParser {

    /**
     * @param lineReader lines read from the output of the z3 program; messages are logged as error, warn or info
     * @throws IOException BufferedReader requires IOException
     */
    public void parse(BufferedReader lineReader) throws IOException {
        boolean lastStatementSat = false;
        boolean lastEventSatisfiable = false;

        String nextLine = lineReader.readLine();

        while (nextLine != null) {
            if (nextLine.equals("sat")) {
                lastStatementSat = true;
            } else if (nextLine.equals("unsat")) {
                lastStatementSat = false;
            } else if (nextLine.startsWith("(error")) {
                Log.error("0xCC003 INTERNAL ERROR:" + getInternalError(nextLine));
            } else if (isInitiallySatisfiable(nextLine) && lastStatementSat == false) {
                Log.error("0xCC001 Initial configuration of component contains errors. Please check ranges of ports");
                return;
            } else if (isMultipleAccess(nextLine) && lastStatementSat == true) {
                for (AbstractMap.SimpleEntry<String, String[]> entry : getErrorMessageForMultipleAccess(nextLine)) {
                    Log.error(String.format("0xCC002 Collision of port %1$s in lines: %2$s and %3$s", entry.getKey(), entry.getValue()[0], entry.getValue()[1]));
                }
            } else if (isConditionEquals(nextLine) && lastEventSatisfiable == true && lastStatementSat == false) {
                String[] messages = getErrorMessageForConditionEquality(nextLine);
                Log.warn(String.format("0xCC004 Conditions in lines %1$s and %2$s are the same", messages[0], messages[1]));
            } else if (isPreamble(nextLine)) {
                jline.internal.Log.info(String.format("Checking consistency of component '%1$s'", getParameter("value", nextLine)));
            } else if (isBaseCondNotSat(nextLine)) {
                lastEventSatisfiable = lastStatementSat;
                if (lastStatementSat == false) {
                    Log.warn(String.format("0xCC005 Condition in line '%1$s' is not satisfiable", getParameter("line", nextLine)));
                }
            } else if (isInvalidBoundary(nextLine) && lastStatementSat == false) {
                Log.error(String.format("0xCC007 Connected value in line '%1$s' violates the range of the target port", getParameter("line", nextLine)));
            }
            nextLine = lineReader.readLine();
        }
    }

    private static boolean isPreamble(String source) {
        return getMessageType(Enums.LogMessageTypes.PREAMBLE, source);
    }

    private static boolean isInitiallySatisfiable(String source) {
        return getMessageType(Enums.LogMessageTypes.INITIALLYSATISFIABLE, source);
    }

    private static boolean isMultipleAccess(String source) {
        return getMessageType(Enums.LogMessageTypes.PORTOVERLAP, source);
    }

    private static boolean isConditionEquals(String source) {
        return getMessageType(Enums.LogMessageTypes.EQUALCONDITIONS, source);
    }

    private static boolean isBaseCondNotSat(String source) {
        return getMessageType(Enums.LogMessageTypes.SINGLECONDITIONNOTSATISFIABLE, source);
    }

    private static boolean isInvalidBoundary(String source) {
        return getMessageType(Enums.LogMessageTypes.INVALIDBOUNDARY, source);
    }

    private static boolean getMessageType(Enums.LogMessageTypes type, String source) {
        Matcher matcher = Pattern.compile(type + ":+.*").matcher(source);
        return matcher.find();
    }

    private static String getParameter(String type, String source) {
        Matcher matcher = Pattern.compile(".*" + type + "=(.*?);.*").matcher(source);

        if (matcher.find())
            return matcher.group(1);

        return "";
    }

    private static String getInternalError(String source) {
        Matcher matcher = Pattern.compile("\\(error \"(.*?)\"").matcher(source);

        if (matcher.find())
            return matcher.group(1);

        return "";
    }

    private static List<AbstractMap.SimpleEntry<String, String[]>> getErrorMessageForMultipleAccess(String source) {
        List<AbstractMap.SimpleEntry<String, String[]>> retVal = new ArrayList<>();

        Matcher multipleAccessMatcher = Pattern.compile("\\{(.*?)\\};").matcher(source);

        while (multipleAccessMatcher.find()) {
            String foundString = multipleAccessMatcher.group(1);
            retVal.add(new AbstractMap.SimpleEntry<String, String[]>(foundString.split(",")[0], new String[]{foundString.split(",")[1], foundString.split(",")[2]}));//multipleAccessMatcher.group(1), new HashMap<>());
        }
        return retVal;
    }

    private static String[] getErrorMessageForConditionEquality(String source) {
        String[] retVal = new String[2];

        Matcher multipleAccessMatcher = Pattern.compile("\\{(.*?)\\};").matcher(source);

        while (multipleAccessMatcher.find()) {
            String foundString = multipleAccessMatcher.group(1);
            retVal[0] = foundString.split(",")[0];
            retVal[1] = foundString.split(",")[1];
        }
        return retVal;
    }
}