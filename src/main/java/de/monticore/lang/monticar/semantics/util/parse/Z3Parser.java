/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.parse;

import de.monticore.lang.monticar.semantics.solver.solutionValidation.result.ArithmeticValidationResult;
import de.monticore.lang.monticar.semantics.solver.solutionValidation.result.ArithmeticValidationResultBuilder;
import de.monticore.lang.monticar.semantics.solver.solutionValidation.result.PortRangeValidationResult;
import de.monticore.lang.monticar.semantics.solver.solutionValidation.result.PortRangeValidationResultBuilder;
import de.monticore.lang.monticar.semantics.underspecification.SolveResult;
import de.monticore.lang.monticar.semantics.underspecification.SolveResultBuilder;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Z3Parser {

    public static SolveResult parseSolve(String output) {
        SolveResultBuilder builder = SolveResultBuilder.aSolveResult();

        BufferedReader reader = new BufferedReader(new StringReader(output));
        try {
            builder.setSatisfiable("sat".equals(reader.readLine()));
            builder.setModel(parseModel(reader));
        } catch (IOException e) { }

        return builder.build();
    }

    public static List<PortRangeValidationResult> parsePortRangeValidations(String output) {
        List<PortRangeValidationResult> result = new ArrayList<>();
        BufferedReader reader = new BufferedReader(new StringReader(output));

        try {
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.equals("Port Range Validation"))
                    result.add(parsePortRangeValidation(reader));
            }
        } catch (IOException e) {

        }
        return result;
    }

    public static List<ArithmeticValidationResult> parseArithmeticValidations(String output) {
        List<ArithmeticValidationResult> result = new ArrayList<>();
        BufferedReader reader = new BufferedReader(new StringReader(output));

        try {
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.equals("Arithmetic Validation"))
                    result.add(parseArithmeticValidation(reader));
            }
        } catch (IOException e) {

        }
        return result;
    }

    private static PortRangeValidationResult parsePortRangeValidation(BufferedReader reader) {
        PortRangeValidationResultBuilder builder = PortRangeValidationResultBuilder.aPortRangeValidation();
        try {
            String line = reader.readLine();
            builder.setName(getName(line));
            builder.setUpper(isUpper(line));
            builder.setSatisfiable("sat".equals(reader.readLine()));
            builder.setModel(parseModel(reader));
        } catch (IOException e) { }

        return builder.build();
    }


    public static ArithmeticValidationResult parseArithmeticValidation(BufferedReader reader) {
        ArithmeticValidationResultBuilder builder = ArithmeticValidationResultBuilder.anArithmeticValidation();
        try {
            builder.setExpression(getName(reader.readLine()));
            builder.setSatisfiable("sat".equals(reader.readLine()));
            builder.setModel(parseModel(reader));
        } catch (IOException e) { }

        return builder.build();
    }

    private static Map<String, Double> parseModel(BufferedReader reader) throws IOException {
        Map<String, Double> result = new HashMap<>();
        String line;
        int brackets = 1;
        while ((line = reader.readLine()) != null) {
            brackets += countOccurance(line, '(') - countOccurance(line, ')');
            if (line.contains("(model ")) brackets = 1;
            if (brackets == 0) break;

            if (line.contains("define-fun")) {
                String varName = getNameOfParameter(line);
                line = reader.readLine();
                brackets += countOccurance(line, '(') - countOccurance(line, ')');
                double value = getValue(line);
                result.put(varName, value);
            }
        }

        return result;
    }

    private static long countOccurance(String string, char c) {
        return string.chars().filter(x -> c == x).count();
    }

    private static String getName(String line) {
        return line.split(" ")[0];
    }

    private static boolean isUpper(String line) {
        return line.split(" ")[1].equals("upper");
    }

    private static double getValue(String line) {
        Pattern valuePattern = Pattern.compile("\\s*(\\d+\\.\\d+)\\)");
        Matcher matcher = valuePattern.matcher(line);
        String valueString = "";
        if (matcher.find())
            valueString = matcher.group(1);

        double v = 0;
        if (!"".equals(valueString)) {
             v = Double.parseDouble(valueString);
        }

        return v;
    }

    private static String getNameOfParameter(String line) {
        Pattern nameOfFunctionPattern = Pattern.compile("\\s*\\(define-fun (\\w+) \\(\\).*");
        Matcher matcher = nameOfFunctionPattern.matcher(line);
        String varName = "";
        if (matcher.find())
            varName = matcher.group(1);
        return varName;
    }
}
