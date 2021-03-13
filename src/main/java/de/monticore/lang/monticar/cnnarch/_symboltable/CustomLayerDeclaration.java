/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import org.apache.commons.lang3.SystemUtils;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;

public class CustomLayerDeclaration extends LayerDeclarationSymbol{

    private String customPythonFilesPath;

    protected CustomLayerDeclaration(String name, String customPythonFilesPath) {
        super(name);
        setCustomPythonFilesPath(customPythonFilesPath);
    }

    protected void setParameters(List<ParameterSymbol> parameters) {
        super.setParameters(parameters);
        for (ParameterSymbol param : parameters) {
            param.putInScope(getSpannedScope());
        }
    }

    private void setCustomPythonFilesPath(String customPythonFilesPath){
        this.customPythonFilesPath = customPythonFilesPath;
    }

    public String getCustomPythonFilesPath(){
        return this.customPythonFilesPath;
    }

    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer) {
        String computeOutputPython = "[";
        for (int index = 0; index < inputTypes.size(); index++){
            computeOutputPython += "(" + layer.getInputTypes().get(index).getChannels() +"," + layer.getInputTypes().get(index).getHeight() +"," + layer.getInputTypes().get(index).getWidth() + ")";
            if (index != inputTypes.size()-1){
                computeOutputPython += ",";
            } else {
                computeOutputPython += "]";
            }
        }
        String parametersOfLayer = "";
        for (int index = 0; index < layer.getArguments().size(); index++) {
            parametersOfLayer += layer.getArguments().get(index).getRhs().getValue().get().toString() + ",";
        }

        String outputDimensionsAsString = getOutputDimensionsFromPython(parametersOfLayer + computeOutputPython);
        outputDimensionsAsString = outputDimensionsAsString.replaceAll("\\s", "");
        outputDimensionsAsString = outputDimensionsAsString.replaceAll("'", "");
        String [] outputDimensions = outputDimensionsAsString.substring(1, outputDimensionsAsString.indexOf(")")).split(",");

        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(Integer.parseInt(outputDimensions[0]))
                        .height(Integer.parseInt(outputDimensions[1]))
                        .width(Integer.parseInt(outputDimensions[2]))
                        .elementType(outputDimensions[3], outputDimensions[4])
                        .build());


    }

    private String getOutputDimensionsFromPython(String computeOutputPython) {

        String outputDimensions = "";
        boolean osCheckWindows = SystemUtils.IS_OS_WINDOWS;
        File path = new File(getCustomPythonFilesPath() + "custom_layers/");

        String printOutputDimensions;
        String terminal = "";
        String com = "";
        try {
                String callingPrintMethod = "import " + this.getName() + "; temp=" + this.getName() + "." + this.getName() + "(); temp.compute_output_types(" + computeOutputPython + ")";
                if (osCheckWindows == false) {
                    printOutputDimensions = "python3 -c '" + callingPrintMethod + "'";
                    terminal = "/bin/bash";
                    com = "-c";
                } else {
                    printOutputDimensions = "python -c \"" + callingPrintMethod + "\"";
                    terminal = "cmd.exe";
                    com = "/c";
                }
                Process process = Runtime.getRuntime().exec(new String[]{terminal, com, printOutputDimensions}, null, path);

                String line = null;
                ArrayList<String> hold = new ArrayList<>();
                BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
                while ((line = reader.readLine()) != null) {
                    hold.add(line);
                }
                reader.close();

                if (hold.isEmpty()) {
                    Process process1 = Runtime.getRuntime().exec(new String[]{terminal, com, printOutputDimensions}, null, path);

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



    @Override
    public CustomLayerDeclaration deepCopy() {
        CustomLayerDeclaration copy = new CustomLayerDeclaration(getName(), customPythonFilesPath);
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

    public boolean isCustom(){
        return true;
    }

}
