/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.models;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.generator.ScriptGenerator;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;
import org.apache.commons.cli.ParseException;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ModelSplitterImpl implements ModelSplitter {
    protected final ScriptGenerator generator;
    protected final OptionsService optionsService;

    @Inject
    public ModelSplitterImpl(ScriptGenerator generator, OptionsService optionsService) {
        this.generator = generator;
        this.optionsService = optionsService;
    }

    @Override
    public List<File> split(File model) throws IOException {
        List<File> parts = new ArrayList<>();

        String emamName = model.getName();
        Path directory = model.toPath().getParent();

        String contents = FileUtils.readFileToString(model, "UTF-8");

        /*File emaFile = this.splitToEmbeddedMontiArc(emamName, directory, contents);

        parts.add(emaFile);*/

        if (this.hasMontiMath(contents)) {
            File mFile = this.splitToMontiMath(emamName, directory, contents);

            parts.add(mFile);
        }

        return parts;
    }

    /*protected File splitToEmbeddedMontiArc(String model, Path directory, String contents) throws IOException {
        String emaName = model.replace(".emam", ".ema");
        File emaFile = directory.resolve(emaName).toFile();

        contents = this.removeComments(contents);
        contents = this.hasMontiMath(contents) ? this.removeMontiMath(contents) : contents;

        FileUtils.writeStringToFile(emaFile, contents, "UTF-8");

        return emaFile;
    }*/

    protected File splitToMontiMath(String model, Path directory, String contents) throws IOException {
        String name = model.replace(".emam", "");
        String mName = model.replace(".emam", ".m");
        File mFile = directory.resolve(mName).toFile();
        String packige = this.extractPackage(contents);

        contents = this.removeComments(contents);
        contents = this.extractMontiMath(contents);
        contents = this.generator.generate(packige, name, contents);

        FileUtils.writeStringToFile(mFile, contents, "UTF-8");

        return mFile;
    }

    protected String removeComments(String contents) {
        return contents.replaceAll("((['\"])(?:(?!\\2|\\\\).|\\\\.)*\\2)|\\/\\/[^\\n]*|\\/\\*(?:[^*]|\\*(?!\\/))*\\*\\/", "");
    }

    protected String extractPackage(String contents) {
        Pattern pattern = Pattern.compile("package (.+?);");

        contents = this.removeComments(contents);

        Matcher matcher = pattern.matcher(contents);

        if (matcher.find()) return matcher.group(1);
        else return "";
    }

    protected boolean hasMontiMath(String contents) {
        contents = this.removeComments(contents);

        int start = contents.indexOf("implementation");
        int middle = start > -1 ? contents.indexOf("Math", start) : -1;

        return middle != -1;
    }

    protected Vector<Integer> getMontiMathRange(String contents) {
        char[] chars = contents.toCharArray();
        Vector<Integer> result = new Vector<>();
        int start = contents.indexOf("implementation");
        boolean inBody = false;
        int bracketCount = 0;
        int end = start;

        do {
            if (chars[end] == '{') {
                inBody = true;
                bracketCount++;
            }
            if (chars[end] == '}') bracketCount--;
            if (inBody && bracketCount == 0) break;
        } while (end++ < contents.length());

        result.add(start);
        result.add(end + 1);

        return result;
    }

    /*protected String removeMontiMath(String contents) {
        StringBuilder builder = new StringBuilder(contents);
        Vector<Integer> range = this.getMontiMathRange(contents);

        builder.delete(range.firstElement(), range.lastElement());

        return builder.toString();
    }*/

    protected String extractMontiMath(String contents) {
        Vector<Integer> range = this.getMontiMathRange(contents);
        int start = contents.indexOf("{", range.firstElement());
        int end = range.lastElement();

        return contents.substring(start + 1, end - 1);
    }
}
