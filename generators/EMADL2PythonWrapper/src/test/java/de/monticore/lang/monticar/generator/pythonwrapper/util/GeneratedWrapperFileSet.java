/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;

/**
 *
 */
public class GeneratedWrapperFileSet {
    private final String componentName;

    private String wrapperHeader;
    private String wrapperCpp;
    private String swigInterface;
    private String cmake;

    private GeneratedWrapperFileSet(String componentName) {
        this.componentName = componentName;
    }

    private void setWrapperHeader(String wrapperHeader) {
        this.wrapperHeader = wrapperHeader;
    }

    private void setWrapperCpp(String wrapperCpp) {
        this.wrapperCpp = wrapperCpp;
    }

    private void setSwigInterface(String swigInterface) {
        this.swigInterface = swigInterface;
    }

    private void setCmake(String cmake) {
        this.cmake = cmake;
    }

    public String getComponentName() {
        return componentName;
    }

    public String getWrapperHeader() {
        return wrapperHeader;
    }

    public String getWrapperCpp() {
        return wrapperCpp;
    }

    public String getSwigInterface() {
        return swigInterface;
    }

    public String getCmake() {
        return cmake;
    }

    private static String readContentFromUrl(URL url) throws IOException {
        StringBuilder content = new StringBuilder();
        BufferedReader reader = new BufferedReader(new InputStreamReader(url.openStream()));
        int input;
        while((input = reader.read()) != -1) {
            content.append((char)input);
        }
        return content.toString();
    }

    public static GeneratedWrapperFileSet fromNameAndUrl(String componentName, URL urlToFiles) throws IOException {
        GeneratedWrapperFileSet set = new GeneratedWrapperFileSet(componentName);

        URL urlToHeader = new URL(urlToFiles, urlToFiles.getFile() + "/" + componentName + "_executor.h");
        URL urlToCpp = new URL(urlToFiles, urlToFiles.getFile() + "/" + componentName + "_executor.cpp");
        URL urlToSwigInterface = new URL(urlToFiles, urlToFiles.getFile() + "/" + componentName + "_executor.i");
        URL urlToCmake = new URL(urlToFiles, urlToFiles.getFile() + "/" + "CMakeLists.txt");

        set.setWrapperHeader(readContentFromUrl(urlToHeader));
        set.setWrapperCpp(readContentFromUrl(urlToCpp));
        set.setSwigInterface(readContentFromUrl(urlToSwigInterface));
        set.setCmake(readContentFromUrl(urlToCmake));

        return set;
    }
}
