package de.monticore.reporting.svgTools;

import de.monticore.reporting.helper.CommonModelInfo;
import de.monticore.reporting.helper.OrderableModelInfo;
import de.monticore.reporting.order.ChildElement;

import java.io.File;
import java.util.LinkedList;
import java.util.List;

public abstract class SVGInfo extends OrderableModelInfo {

    private String svgPath = "";
    private boolean emaModelPathAvailable;
    private String emaModelPath = "";
    private String emaToPrint = "";

    public SVGInfo(File modelFile){
        super(modelFile);
    }

    public SVGInfo(String model) {
        super(model);
    }

    public String getSvgPath() {
        return svgPath;
    }

    public void setSvgPath(String svgPath) {
        this.svgPath = svgPath;
    }

    public boolean isEmaModelPathAvailable() {
        return emaModelPathAvailable;
    }

    public void setEmaModelPathAvailable(boolean emaModelPathAvailable) {
        this.emaModelPathAvailable = emaModelPathAvailable;
    }

    public String getEmaModelPath() {
        return emaModelPath;
    }

    public void setEmaModelPath(String emaModelPath) {
        this.emaModelPath = emaModelPath;
        if(!emaModelPath.equals(""))
            setEmaModelPathAvailable(true);
    }

    public String getEmaToPrint() {
        return emaToPrint;
    }

    public void setEmaToPrint(String emaToPrint) {
        this.emaToPrint = emaToPrint;
    }
}
