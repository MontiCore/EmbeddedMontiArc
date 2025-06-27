/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;
import org.apache.commons.lang3.SystemUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class CustomCPPLayerDeclaration extends  CustomLayerDeclaration{

    public CustomCPPLayerDeclaration(String name, File customFilePath, String language) {
        super(name, customFilePath, language);
    }

    @Override
    public ArrayList<ParameterSymbol> extractParametersFromFile(){

        String compileLayerFile;
        String executeFile;
        String terminal;
        String com;
        try {
            if (SystemUtils.IS_OS_WINDOWS == false) {
                compileLayerFile = "g++ -std=c++11 " + getName() + ".cpp -o" + getName();
                executeFile = "./" + getName() + " p";
                terminal = "/bin/bash";
                com = "-c";
            } else {
                compileLayerFile = "gcc -std=c++11 " + getName() + ".cpp -o" + getName();
                executeFile = "./" + getName() + " p";
                terminal = "cmd.exe";
                com = "/c";
            }
            Process compileProcess = Runtime.getRuntime().exec(new String[]{terminal, com, compileLayerFile}, null, getCustomFilePath());
            Process process = Runtime.getRuntime().exec(new String[]{terminal, com, executeFile}, null, getCustomFilePath());

            String line = null;
            ArrayList<String> hold = new ArrayList<>();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            while ((line = reader.readLine()) != null) {
                hold.add(line);
            }
            reader.close();

            if (hold.isEmpty()) {
                Process process1 = Runtime.getRuntime().exec(new String[]{terminal, com, compileLayerFile, ";", executeFile}, null, getCustomFilePath());

                String error = "";
                BufferedReader errorReader = new BufferedReader(new InputStreamReader(process1.getErrorStream()));
                while ((line = errorReader.readLine()) != null) {
                    error += line;
                }
                errorReader.close();
                throw new IOException(error);
            } else if ((hold.get(0)).equals("{}")) {
                return new ArrayList<ParameterSymbol>();
            } else {
                String mergeLines = "";
                for (int numb = 0; numb < hold.size(); numb++) {
                    mergeLines += hold.get(numb);
                }
                return createParameterListFromString(processAttributesString(mergeLines));
            }
        } catch (IOException e){
            e.printStackTrace();
        }
        return null;
    }

    private ArrayList<String> processAttributesString(String attributeArray){
        attributeArray = attributeArray.substring(1, attributeArray.length()-1).replaceAll("\\s", "");
        attributeArray = attributeArray.replaceAll("\\(", "").replaceAll("\\)", "");

        return new ArrayList<>(Arrays.asList(attributeArray.split(",")));
    }

    private ArrayList<ParameterSymbol> createParameterListFromString(ArrayList<String> paramsAsString){
        ArrayList<ParameterSymbol> parameters = new ArrayList<ParameterSymbol>();

        if(!(paramsAsString.isEmpty())){
            for(int index = 0; index < paramsAsString.size(); index += 2){
                switch (paramsAsString.get(index+1)){
                    case "int":
                        parameters.add(new ParameterSymbol.Builder()
                                .name(paramsAsString.get(index))
                                .constraints(Constraints.INTEGER)
                                .defaultValue(0)
                                .build());
                        break;
                    case "float":
                    case "complex":
                    case "long":
                        parameters.add(new ParameterSymbol.Builder()
                                .name(paramsAsString.get(index))
                                .constraints(Constraints.NUMBER)
                                .defaultValue(0)
                                .build());
                        break;
                    case "tuple":
                        parameters.add(new ParameterSymbol.Builder()
                                .name(paramsAsString.get(index))
                                .constraints(Constraints.INTEGER_TUPLE)
                                .defaultValue(Arrays.asList(1,1))
                                .build());
                        break;
                    case "bool":
                        parameters.add(new ParameterSymbol.Builder()
                                .name(paramsAsString.get(index))
                                .constraints(Constraints.BOOLEAN)
                                .defaultValue(false)
                                .build());
                        break;
                    case "string":
                        parameters.add(new ParameterSymbol.Builder()
                                .name(paramsAsString.get(index))
                                .constraints(Constraints.STRING)
                                .defaultValue("")
                                .build());
                        break;
                    default:
                        parameters.add(new ParameterSymbol.Builder()
                                .name(paramsAsString.get(index))
                                .defaultValue("")
                                .build());
                }
            }
        }
        return parameters;
    }



    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        String outputDimensionsAsString = "";

        String compileLayerFile;
        String executeFile;
        String terminal;
        String com;
        if (SystemUtils.IS_OS_WINDOWS == false) {
            compileLayerFile = "g++ -std=c++11 " + getName() + ".cpp -o" + getName();
            executeFile = "./" + getName() + " o";
            terminal = "/bin/bash";
            com = "-c";
        } else {
            compileLayerFile = "g++ -std=c++11 " + getName() + ".cpp -o" + getName();
            executeFile = "./" + getName() + " o";
            terminal = "/bin/bash";
            com = "-c";
        }

        try {
            Process compileProcess = Runtime.getRuntime().exec(new String[] {terminal, com, compileLayerFile}, null, getCustomFilePath());
            Process process = Runtime.getRuntime().exec(new String[]{terminal, com, executeFile}, null, getCustomFilePath());

            String line = null;
            ArrayList<String> hold = new ArrayList<>();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            while ((line = reader.readLine()) != null) {
                hold.add(line);
            }
            reader.close();

            if (hold.isEmpty()) {
                Process process1 = Runtime.getRuntime().exec(new String[]{terminal, com, compileLayerFile, ";", executeFile}, null, getCustomFilePath());

                String error = "";
                BufferedReader errorReader = new BufferedReader(new InputStreamReader(process1.getErrorStream()));
                while ((line = errorReader.readLine()) != null) {
                    error += line;
                }
                errorReader.close();
                throw new IOException(error);

            } else {
                for (int index = 0; index < hold.size(); index++){
                    outputDimensionsAsString += hold.get(index);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        ArrayList<String []> outputDimensions = processStringFromTerminal(outputDimensionsAsString);

        List<ArchTypeSymbol> outputShapes  = new ArrayList<>();
        for (String[] outputDim : outputDimensions){
            outputShapes.add(
                    new ArchTypeSymbol.Builder()
                            .channels(Integer.parseInt(outputDim[0]))
                            .height(Integer.parseInt(outputDim[1]))
                            .width(Integer.parseInt(outputDim[2]))
                            .elementType(outputDim[3], outputDim[4])
                            .build());
        }

        return outputShapes;
    }




    private ArrayList<String []> processStringFromTerminal (String str){
        boolean check = true;
        ArrayList<String []> inputs = new ArrayList<String []>();

        str = str.substring(1, str.length()-1).replaceAll("\\s", "");

        while(check == true){

            if(str.indexOf(")") != -1){
                inputs.add((str.substring(1, str.indexOf(")"))).split(","));
            } else {
                check = false;
            }

            if(str.indexOf(")") == str.length()-1){
                check = false;
            } else {
                str = str.substring(str.indexOf(")")+2);
            }
        }

        return inputs;
    }

    public CustomCPPLayerDeclaration deepCopy() {
        CustomCPPLayerDeclaration copy = new CustomCPPLayerDeclaration(getName(), getCustomFilePath(), getLanguage());
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        if (this.getParameters() != null) {
            List<ParameterSymbol> parameterCopies = new ArrayList<>(getParameters().size());
            for (ParameterSymbol parameter : getParameters()) {
                ParameterSymbol parameterCopy = parameter.deepCopy();
                parameterCopies.add(parameterCopy);
                parameterCopy.putInScope(copy.getSpannedScope());
            }
            copy.setParameters(parameterCopies);
        }
        return copy;
    }
}
