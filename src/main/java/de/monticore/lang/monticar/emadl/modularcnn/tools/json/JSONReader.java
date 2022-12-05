/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.tools.json;

import de.monticore.lang.monticar.emadl.modularcnn.compositions.NetworkStructureInformation;

import java.util.ArrayList;

public class JSONReader {

    public JSONReader() {}

    public NetworkStructureInformation getNetworkStructureInstance(String jsonContent) {
        if (!validJSON(jsonContent)) return null;
        return getLevelInstance(jsonContent, 0, jsonContent.length() - 1);
    }

    private NetworkStructureInformation getLevelInstance(String jsonContent, int start, int end) {
        jsonContent = removeNewLinesAndWhitespaces(jsonContent);
        int[] currentLevel = findTopLevelType(jsonContent, start, end);
        NetworkStructureInformation networkStructureInformation = processObject(jsonContent.substring(currentLevel[0], currentLevel[1]));

        return networkStructureInformation;
    }

    private NetworkStructureInformation processObject(String jsonContent) {
        ArrayList<String> separatedEntries = separateEntries(jsonContent);
        ArrayList<JSONKVPair> levelKvPairs = splitLevelContent(separatedEntries);

        String name = null;
        String instanceName = null;
        boolean atomic = false;
        ArrayList<String> dataFlow = null;
        ArrayList<NetworkStructureInformation> subnets = null;

        for (int i = 0; i < levelKvPairs.size(); i++) {
            switch (levelKvPairs.get(i).key) {
                case "name":
                    name = levelKvPairs.get(i).value;
                    break;
                case "instanceSymbolName":
                    instanceName = levelKvPairs.get(i).value;
                    break;
                case "atomic":
                    atomic = Boolean.parseBoolean(levelKvPairs.get(i).value);
                    break;
                case "dataFlow":
                    String arrayContentDataFlow = levelKvPairs.get(i).value;
                    int[] levelDF = findTopLevelType(arrayContentDataFlow, 0, arrayContentDataFlow.length() - 1);
                    dataFlow = processStringArray(arrayContentDataFlow.substring(levelDF[0],levelDF[1]));
                    break;
                case "subNetworks":
                    String arrayContent = levelKvPairs.get(i).value;
                    int[] level = findTopLevelType(arrayContent, 0, arrayContent.length() - 1);
                    subnets = processArray(arrayContent.substring(level[0], level[1]));
                    break;
                default:
                    break;
            }
        }

        NetworkStructureInformation networkStructureInformation = new NetworkStructureInformation(name, instanceName, atomic, subnets, dataFlow,null);
        return networkStructureInformation;
    }

    private ArrayList<NetworkStructureInformation> processArray(String jsonContent) {
        if (jsonContent.equals("") ) return null;

        ArrayList<String> seperatedEntries = separateEntries(jsonContent);
        ArrayList<NetworkStructureInformation> subNetworkStructures = new ArrayList<>();

        for (int i = 0; i < seperatedEntries.size(); i++) {
            String arrayContent = seperatedEntries.get(i);
            int[] level = findTopLevelType(arrayContent, 0, arrayContent.length() - 1);
            NetworkStructureInformation networkStructureInformation = processObject(arrayContent.substring(level[0], level[1]));
            subNetworkStructures.add(networkStructureInformation);
        }

        if (subNetworkStructures.size() == 0) return null;
        return subNetworkStructures;
    }

    private ArrayList<String> processStringArray(String jsonContent) {
        if (jsonContent.equals("") ) return null;

        ArrayList<String> seperatedEntries = separateEntries(jsonContent);
        ArrayList<String> dataFlow = new ArrayList<>();

        for (int i = 0; i < seperatedEntries.size(); i++) {
            String arrayContent = seperatedEntries.get(i);
            arrayContent = arrayContent.replaceAll("\"","");
            //int[] level = findTopLevelType(arrayContent, 0, arrayContent.length() - 1);
            //String entry = String.valueOf(processStringArray(arrayContent.substring(level[0], level[1])));
            dataFlow.add(arrayContent);
        }

        if (dataFlow.size() == 0) return null;
        return dataFlow;
    }

    private ArrayList<JSONKVPair> splitLevelContent(ArrayList<String> seperatedEntries) {
        ArrayList<JSONKVPair> jsonKvPairs = new ArrayList<>();

        for (int i = 0; i < seperatedEntries.size(); i++) {
            jsonKvPairs.add(extractKeyValuePair(seperatedEntries.get(i)));
        }
        return jsonKvPairs;
    }

    private ArrayList<String> separateEntries(String jsonContent) {

        ArrayList<String> entries = new ArrayList<>();
        char[] chars = jsonContent.toCharArray();

        int start = 0;
        int insideObjectOrArray = 0;

        for (int i = start; i< chars.length;i++){

                if (chars[i] == '{' || chars[i] == '[') insideObjectOrArray++;
                else if (chars[i] == '}' || chars[i] == ']') insideObjectOrArray--;


            if ( (chars[i] == ',' || i == chars.length - 1)  && insideObjectOrArray == 0){
                if (i == chars.length-1) entries.add(jsonContent.substring(start));
                else entries.add(jsonContent.substring(start, i));
                start = i + 1;
            }
        }
        return entries;


    }

    private JSONKVPair extractKeyValuePair(String jsonEntry) {
        String[] pair = jsonEntry.split(":",2);
        String key = pair[0].replace("\"", "");
        String value = "";
        JSONKVPair JSONKVPair = null;

        char[] valueChars = pair[1].toCharArray();
        char startsWith = Character.MIN_VALUE, endsWith = Character.MIN_VALUE;

        for (int i = 0; i < valueChars.length; i++) {
            if (valueChars[i] != Character.MIN_VALUE) {
                startsWith = valueChars[i];
                break;
            }
        }

        for (int i = valueChars.length - 1; i >= 0; i--) {
            if (valueChars[i] != Character.MIN_VALUE) {
                endsWith = valueChars[i];
                break;
            }
        }

        if (startsWith == '[' && endsWith == ']') {
            value = pair[1];
            JSONKVPair = new JSONKVPair(key, value, LevelType.ARRAY);
        } else if (startsWith == '{' && endsWith == '}') {
            value = pair[1];
            JSONKVPair = new JSONKVPair(key, value, LevelType.OBJECT);
        } else {
            value = pair[1].replace("\"", "");
            JSONKVPair = new JSONKVPair(key, value, LevelType.VALUE);
        }

        return JSONKVPair;
    }

    private LevelType translateLevelType(int l) {
        switch (l) {
            case 0:
                return LevelType.ARRAY;
            case 1:
                return LevelType.OBJECT;
            case 3:
                return LevelType.VALUE;
            default:
                return null;
        }
    }

    private int[] findTopLevelType(String jsonContent, int start, int end) {
        char[] chars = jsonContent.toCharArray();
        int left = 0, right = 0;
        char firstHit = Character.MIN_VALUE;
        LevelType foundType = null;
        boolean objectOrArray = false;

        int typeStart = 0, typeEnd = 0;
        int lastTypeEndHit = 0;

        for (int i = start; i <= end; i++) {
            if (firstHit == '{') {
                if (chars[i] == '{') {
                    left++;
                } else if (chars[i] == '}') {
                    right++;
                    lastTypeEndHit = i;
                }
            } else if (firstHit == '[') {
                if (chars[i] == '[') {
                    left++;
                } else if (chars[i] == ']') {
                    right++;
                    lastTypeEndHit = i;
                }
            } else {
                if (chars[i] == '{') {
                    firstHit = '{';
                    foundType = LevelType.OBJECT;
                    typeStart = i + 1;
                } else if (chars[i] == '[') {
                    firstHit = '[';
                    foundType = LevelType.ARRAY;
                    typeStart = i + 1;
                }
            }

            if (firstHit != Character.MIN_VALUE /*&& left == right*/) {
                typeEnd = lastTypeEndHit;
                objectOrArray = true;

            }
        }
        if (objectOrArray) return new int[]{typeStart, typeEnd, foundType == LevelType.ARRAY ? 0 : 1};
        return new int[]{-1, -1, -1};
    }

    private boolean validJSON(String jsonContent) {
        return braceMatch(jsonContent);
    }

    private boolean braceMatch(String jsonContent) {
        int leftBrace = 0, rightBrace = 0, leftBracket = 0, rightBracket = 0;
        char[] chars = jsonContent.toCharArray();
        for (int i = 0; i < chars.length; i++) {
            switch (chars[i]) {
                case '{':
                    leftBrace++;
                    break;
                case '}':
                    rightBrace++;
                    break;
                case '[':
                    leftBracket++;
                    break;
                case ']':
                    rightBracket++;
                    break;
                default:
                    break;
            }
        }
        return leftBrace == rightBrace && leftBracket == rightBracket;
    }

    private String removeNewLinesAndWhitespaces(String json) {
        return json.replaceAll("\n", "").replaceAll(" ", "");
    }
}
