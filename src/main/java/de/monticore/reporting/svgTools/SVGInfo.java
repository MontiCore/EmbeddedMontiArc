package de.monticore.reporting.svgTools;

import de.monticore.reporting.order.ChildElement;

import java.io.File;
import java.util.List;

public interface SVGInfo {

    public String getPathToFile();

    public String getProject();

    public File getRootFile();

    public String getModelName();

    public String getModelPath();

    public String getQualifiedName();

    public String getFileType();

    public int getParsed();

    public int getResolved();

    public String getSvgPath();

    public void setSvgPath(String s);

    public boolean isEmaModelPathAvailable();

    public String getEmaModelPath();

    public void setEmaModelPath(String newModelPath);

    public void setEmaToPrint(String emaString);

    public String getEmaToPrint();

    public List<ChildElement> getChildren();

    public void addErrorMessage(String msg);
}
