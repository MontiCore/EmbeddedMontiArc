package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.List;

public final class InputPort extends ViewModelBase implements IInputPort{
    boolean isCube;

    private String input;

    private List<String> inputList;
    private int n_rows;
    private int n_cols;
    private int n_slices;
    private boolean isImage = false;
    private String imagePath;
    private final String workingDirectory = System.getProperty("user.dir");

    public boolean getIsCube() { return isCube;}

    public String getInput() { return input;}

    public List<String> getInputList() {return inputList;}

    public void setN_cols(int n_cols) { this.n_cols = n_cols;}
    public void setN_rows(int n_rows) { this.n_rows = n_rows;}
    public void setN_slices(int n_slices) { this.n_slices = n_slices;}

    public int getN_rows() {return n_rows;}
    public int getN_cols() {return n_cols;}
    public int getN_slices() {return n_slices;}

    public static InputPort from(Boolean value) {
        return from(Boolean.toString(value));
    }

    public static InputPort from(Double value) {
        return from(Double.toString(value));
    }

    public static InputPort from(String value) {
        InputPort result = new InputPort();
        result.input = value;
        result.isCube = false;
        return result;
    }

    public static InputPort from(List<String> value, int n_slices, int n_rows, int n_cols) {
        InputPort result = new InputPort();
        result.inputList = value;
        result.isCube = true;
        result.n_cols = n_cols;
        result.n_rows = n_rows;
        result.n_slices = n_slices;
        return result;
    }

    public static InputPort from(String imagePath, boolean isImage) {
        InputPort result = new InputPort();
        result.isCube = false;
        result.isImage = isImage;
        result.imagePath = imagePath;
        return result;
    }

    public boolean isImage() {
        return isImage;
    }

    public void setImage(boolean image) {
        isImage = image;
    }

    public String getImagePath() {
        return imagePath;
    }

    public void setImagePath(String imagePath) {
        this.imagePath = imagePath;
    }

    public String getWorkingDirectory() {
        return workingDirectory;
    }
}
