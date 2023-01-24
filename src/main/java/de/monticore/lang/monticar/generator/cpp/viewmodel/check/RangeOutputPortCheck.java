/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FilenameUtils;

import java.io.File;
import java.nio.file.Files;
import java.util.List;
import java.util.Objects;

public final class RangeOutputPortCheck extends ViewModelBase implements IOutputPortCheck {

    private String lowerBound; // TODO: migrate to Link<String>
    private List<String> lowerBoundList;
    private String upperBound;
    private String imagePath;
    private String imagePathWithoutExtension;
    private final String workingDirectory = System.getProperty("user.dir");

    private List<String> upperBoundList;
    boolean isMatrix = false;
    boolean isCube = false;
    boolean isImage = false;
    double elementTolerance = 0;
    double generalTolerance = 0;

    private int n_rows;
    private int n_cols;
    private int n_slices;

    public int getN_rows() { return n_rows;}
    public int getN_cols() { return n_cols;}
    public int getN_slices() { return n_slices;}
    public void setN_rows(int n_rows) { this.n_rows = n_rows; }
    public void setN_cols(int n_cols) { this.n_cols = n_cols; }
    public void setN_slices(int n_slices) { this.n_slices = n_slices;}

    public boolean isMatrix() {
        return isMatrix;
    }

    public void setMatrix(boolean matrix) {
        isMatrix = matrix;
    }

    public void setCube(boolean cube) { isCube = cube; }
    public boolean isCube() {return isCube;}
    public boolean isImage() { return isImage;}

    public String getLowerBound() {
        return lowerBound;
    }

    public void setLowerBound(String lowerBound) {
        this.lowerBound = lowerBound;
    }

    public String getUpperBound() {
        return upperBound;
    }

    public void setUpperBound(String upperBound) {
        this.upperBound = upperBound;
    }

    public List<String> getLowerBoundList() {
        return lowerBoundList;
    }

    public void setLowerBoundList(List<String> lowerBoundList) {
        this.lowerBoundList = lowerBoundList;
    }

    public List<String> getUpperBoundList() {
        return upperBoundList;
    }

    public void setImagePath(String imagePath) { this.imagePath = imagePath; }

    public String getImagePath() { return this.imagePath;}

    public void setUpperBoundList(List<String> upperBoundList) {
        this.upperBoundList = upperBoundList;
    }
    public void setElementTolerance(double elementTolerance) { this.elementTolerance = elementTolerance;}
    public double getElementTolerance() { return elementTolerance;}
    public void setGeneralTolerance(double generalTolerance) {this.generalTolerance = generalTolerance;}
    public double getGeneralTolerance() { return generalTolerance;}
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o instanceof RangeOutputPortCheck) {
            RangeOutputPortCheck that = (RangeOutputPortCheck) o;
            return equalsTo(that);
        }
        return false;
    }

    @Override
    public int hashCode() {
        int result = lowerBound != null ? lowerBound.hashCode() : 0;
        result = 31 * result + (upperBound != null ? upperBound.hashCode() : 0);
        return result;
    }

    @Override
    public String toString() {
        return "RangeOutputPortCheck{" +
                "lowerBound='" + lowerBound + '\'' +
                ", upperBound='" + upperBound + '\'' +
                '}';
    }

    private boolean equalsTo(RangeOutputPortCheck that) {
        return Objects.equals(lowerBound, that.lowerBound) && Objects.equals(upperBound, that.upperBound);
    }

    public static RangeOutputPortCheck from(double lowerBound, double upperBound) {
        if (lowerBound > upperBound) {
            String msg = String.format("lower bound %s exceeds upper bound %s", lowerBound, upperBound);
            Log.error(msg);
            throw new RuntimeException(msg);
        }
        return from(Double.toString(lowerBound), Double.toString(upperBound));
    }

    public static RangeOutputPortCheck from(String lowerBound, String upperBound) {
        Log.errorIfNull(lowerBound);
        Log.errorIfNull(upperBound);
        RangeOutputPortCheck result = new RangeOutputPortCheck();
        result.setLowerBound(lowerBound);
        result.setUpperBound(upperBound);
        return result;
    }

    public static RangeOutputPortCheck from(String lowerBound, String upperBound, boolean isMatrix) {
        Log.errorIfNull(lowerBound);
        Log.errorIfNull(upperBound);
        RangeOutputPortCheck result = new RangeOutputPortCheck();
        result.isMatrix = isMatrix;
        result.setLowerBound(lowerBound);
        result.setUpperBound(upperBound);
        return result;
    }

    public static RangeOutputPortCheck from(List<String> lowerBoundList,
                                            List<String> upperBoundList,
                                            int n_slices, int n_rows, int n_cols,
                                            double elementTolerance, double generalTolerance) {
        Log.errorIfNull(lowerBoundList);
        Log.errorIfNull(upperBoundList);
        RangeOutputPortCheck result = new RangeOutputPortCheck();
        result.isMatrix = true;
        result.isCube = true;
        result.n_cols = n_cols;
        result.n_rows = n_rows;
        result.n_slices = n_slices;
        result.setLowerBoundList(lowerBoundList);
        result.setUpperBoundList(upperBoundList);
        result.setElementTolerance(elementTolerance);
        result.setGeneralTolerance(generalTolerance);
        return result;
    }

    public static RangeOutputPortCheck from(String filePath, double elementTolerance, double generalTolerance) {
        Log.errorIfNull(filePath);
        RangeOutputPortCheck result = new RangeOutputPortCheck();
        result.imagePath = filePath;
        result.isImage = true;
        result.imagePathWithoutExtension = FilenameUtils.removeExtension(FilenameUtils.getName(filePath));
        result.elementTolerance = elementTolerance;
        result.generalTolerance = generalTolerance;
        return result;
    }

    public String getImagePathWithoutExtension() {
        return imagePathWithoutExtension;
    }

    public void setImagePathWithoutExtension(String imagePathWithoutExtension) {
        this.imagePathWithoutExtension = imagePathWithoutExtension;
    }

    public String getWorkingDirectory() {
        return workingDirectory;
    }
}
