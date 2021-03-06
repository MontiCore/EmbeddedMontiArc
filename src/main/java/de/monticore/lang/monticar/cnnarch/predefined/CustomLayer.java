/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import jline.internal.Nullable;
import org.apache.commons.lang3.SystemUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.Parameter;
import java.util.*;


public class CustomLayer extends PredefinedLayerDeclaration {

    private CustomLayer() { super(AllPredefinedLayers.CUSTOM_LAYER);
    }


    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        return Collections.singletonList(
                new ArchTypeSymbol.Builder()
                        .channels(layer.getInputTypes().get(0).getChannels())
                        .height(layer.getInputTypes().get(0).getHeight())
                        .width(layer.getInputTypes().get(0).getWidth())
                        .elementType("0", "oo")
                        .build());
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    //A method to extract the parameters of the custom layers from their respective .py files
    /*private ArrayList<String> getParameters(String pathToCustomPyFiles){
        ArrayList<String> attributesDict = new ArrayList<>();
        boolean osCheckWindows = SystemUtils.IS_OS_WINDOWS;
        String[] pyFiles;
        File path = new File(pathToCustomPyFiles + "custom_layers/");
        pyFiles = path.list();

        String printParametersOfLayer = "";
        try {

            for(int index = 0; index < pyFiles.length; index++){
                if(pyFiles[index].equals("__init__.py") || !(pyFiles[index].substring(pyFiles[index].length() - 3, pyFiles[index].length())).equals(".py") ) continue;
                String nameWithoutEnding = pyFiles[index].substring(0, pyFiles[index].length() - 3);

                String callingPrintMethod = "import " + nameWithoutEnding + "; temp=" + nameWithoutEnding + "." + nameWithoutEnding + "(); temp.print_parameters_with_type()";
                if(osCheckWindows == false){
                    printParametersOfLayer = "python3 -c '" + callingPrintMethod + "'";
                } else {
                    printParametersOfLayer = "python -c \"" + callingPrintMethod + "\"";
                }
                Process process = Runtime.getRuntime().exec(new String [] {"/bin/bash", "-c", printParametersOfLayer},null, path);

                String line = null;
                BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
                while ((line = reader.readLine()) != null) {
                    attributesDict.add(line);
                }
                reader.close();
            }

        } catch (IOException e) {
            e.printStackTrace();
        }

        ArrayList<String> attributes = new ArrayList<>();

        // {'ones': 'int', 'alpha': 'int'}
        for(int i = 0; i < attributesDict.size(); i++){
            attributes.addAll(processAttributesDict(attributesDict.get(i)));
        }

        return attributes;
    }

    private ArrayList<String> processAttributesDict(String attributesDict){
        String dict = attributesDict;
        dict = dict.replaceAll(",", " ");
        dict = dict.replaceAll("'", " ");
        dict = dict.replaceAll(" ", ":");
        String noBrackets = dict.substring(2, dict.length() - 2);
        String[] processed = noBrackets.split("\\::::");

        return new ArrayList<>(Arrays.asList(processed));
    }
    */


    public static CustomLayer create(){
        CustomLayer declaration = new CustomLayer();
        //ArrayList<String> param = declaration.getParameters("/home/drakeman/mnistcalculator/gluon-cpp/src/python/gluon/");
        //ArrayList<String> param = declaration.getParameters(customPythonFilesPath);
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.CUSTOM_NAME)
                        .defaultValue("")
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.CUSTOM_PARAMETERS)
                        .defaultValue("")
                        .build()));

        declaration.setParameters(parameters);
        return declaration;
    }

}
