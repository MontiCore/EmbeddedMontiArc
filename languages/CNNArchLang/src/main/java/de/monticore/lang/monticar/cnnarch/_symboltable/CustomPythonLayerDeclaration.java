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

public class CustomPythonLayerDeclaration extends CustomLayerDeclaration{

    private String pythonPath = "";

    public CustomPythonLayerDeclaration(String name, File customPythonFilePath, String pythonPath, String language) {
        super(name, customPythonFilePath, language);
        if(pythonPath == ""){
            this.pythonPath = "python";
        } else {
            this.pythonPath = pythonPath;
        }
    }

    //extracting parameters by taking the information from the python file of the layer
    public ArrayList<ParameterSymbol> extractParametersFromFile(){

        String printParametersOfLayer;
        String terminal;
        String com;
        try {
            String callingPrintMethod = "import " + getName() + "; temp=" + getName() + "." + getName() + "(); temp.print_parameters()";
            if (SystemUtils.IS_OS_WINDOWS == false) {
                printParametersOfLayer = this.pythonPath + " -c '" + callingPrintMethod + "'";
                terminal = "/bin/bash";
                com = "-c";
            } else {
                printParametersOfLayer = this.pythonPath + " -c \"" + callingPrintMethod + "\"";
                terminal = "cmd.exe";
                com = "/c";
            }
            Process process = Runtime.getRuntime().exec(new String[]{terminal, com, printParametersOfLayer},null, getCustomFilePath());

            String line = null;
            ArrayList<String> hold = new ArrayList<>();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            while ((line = reader.readLine()) != null) {
                hold.add(line);
            }
            reader.close();

            if (hold.isEmpty()) {
                Process process1 = Runtime.getRuntime().exec(new String[]{terminal, com, printParametersOfLayer}, null, getCustomFilePath());

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
                return createParameterListFromString(processAttributesDict(mergeLines));
            }
        } catch (IOException e){
            e.printStackTrace();
        }
        return null;
    }



    private static ArrayList<String> processAttributesDict(String attributesDict){
        String dict = attributesDict;
        dict = dict.substring(1, dict.length()-1);
        dict = dict.replaceAll(":", ",");
        dict = dict.replaceAll("'", "");
        dict = dict.replaceAll("\\s", "");
        String[] processed = dict.split(",");

        return new ArrayList<>(Arrays.asList(processed));
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
                    case "boolean":
                        parameters.add(new ParameterSymbol.Builder()
                                .name(paramsAsString.get(index))
                                .constraints(Constraints.BOOLEAN)
                                .defaultValue(false)
                                .build());
                        break;
                    case "str":
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

    //computing output types by extracting the information about them from the python file of the layer
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {

        String [] stringForTerminal = createDimensionStringForTerminal(inputTypes, layer);
        String outputDimensionsAsString = getOutputDimensionsFromPython(stringForTerminal);
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

    private String getOutputDimensionsFromPython(String [] stringForTerminal) {
        String outputDimensions = "";

        try {
            Process process = Runtime.getRuntime().exec(stringForTerminal, null, getCustomFilePath());

            String line = null;
            ArrayList<String> hold = new ArrayList<>();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            while ((line = reader.readLine()) != null) {
                hold.add(line);
            }
            reader.close();

            if (hold.isEmpty()) {
                Process process1 = Runtime.getRuntime().exec(stringForTerminal, null, getCustomFilePath());

                String error = "";
                BufferedReader errorReader = new BufferedReader(new InputStreamReader(process1.getErrorStream()));
                while ((line = errorReader.readLine()) != null) {
                    error += line;
                }
                errorReader.close();
                throw new IOException(error);

            } else {
                for (int index = 0; index < hold.size(); index++){
                    outputDimensions += hold.get(index);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return outputDimensions;
    }



    private String [] createDimensionStringForTerminal (List<ArchTypeSymbol> inputTypes, LayerSymbol layer){
        String parametersOfLayer = "";
        for (int index = 0; index < layer.getArguments().size(); index++) {
            parametersOfLayer += layer.getArguments().get(index).getRhs().getValue().get().toString() + ",";
        }

        String computeOutputPython = "[";
        for (int index = 0; index < inputTypes.size(); index++){
            computeOutputPython += "(" + layer.getInputTypes().get(index).getChannels() +"," + layer.getInputTypes().get(index).getHeight() +"," + layer.getInputTypes().get(index).getWidth() + ")";
            if (index != inputTypes.size()-1){
                computeOutputPython += ",";
            } else {
                computeOutputPython += "]";
            }
        }

        String printOutputDimensions = "";
        String terminal = "";
        String com = "";
        String callingPrintMethod = "import " + this.getName() + "; temp=" + this.getName() + "." + this.getName() + "(); temp.compute_output_types(" + parametersOfLayer + computeOutputPython + ")";
        if (SystemUtils.IS_OS_WINDOWS == false) {
            printOutputDimensions = this.pythonPath + " -c '" + callingPrintMethod + "'";
            terminal = "/bin/bash";
            com = "-c";
        } else {
            printOutputDimensions = this.pythonPath + " -c \"" + callingPrintMethod + "\"";
            terminal = "cmd.exe";
            com = "/c";
        }
        String [] stringForTerminal = {terminal, com, printOutputDimensions};

        return stringForTerminal;
    }



    private ArrayList<String []> processStringFromTerminal (String str){
        boolean check = true;
        ArrayList<String []> inputs = new ArrayList<String []>();

        str = str.substring(1, str.length()-1).replaceAll("\\s", "").replaceAll("'","");

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

    public CustomPythonLayerDeclaration deepCopy() {
        CustomPythonLayerDeclaration copy = new CustomPythonLayerDeclaration(getName(), getCustomFilePath(), this.pythonPath, getLanguage());
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


