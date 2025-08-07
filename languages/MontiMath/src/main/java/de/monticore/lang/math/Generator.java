/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.lang.math._cocos.MathCocos;
import de.monticore.lang.math._cocos.MathCoCoChecker;
import de.monticore.lang.math._ast.*;

import java.io.*;
import java.util.*;

/**
 *         generator class that generade the appropriate code
 */
public class Generator extends AbstCocoCheck {

    private String FILEENDING = ".m";
    public static String GENERATEDTARGETPATH = "./target/generated-sources/monticore/sourcecode/de/monticore/lang/math/math/generator/";
    public static String GENERATEDCLASSTARGETPATH = "./target/classes/";
    public static String BASEPACKAGE = "de.monticore.lang.math.generator";
    private int tmpCount = 0;
    private BufferedWriter bw;
    private Stack<String> names = new Stack<>(); // stack to save code fragments for Operations (Numbers)
    private Stack<String> units = new Stack<>(); // stack to save code fragments for Operations (Units)
    private Stack<Object> upnStack = new Stack<>(); //stack for Calculations (reverse Polish Notation)
    private List<String> imports = Arrays.asList("package de.monticore.lang.math.generator;",
            "import org.jscience.mathematics.number.Rational;",
            "import org.jscience.mathematics.vector.DenseMatrix;",
            "import org.jscience.mathematics.vector.Matrix;",
            "import javax.measure.unit.Unit;", "import Executable;",
            "import java.lang.reflect.Field;",
            "import java.io.FileDescriptor;",
            "import java.io.FileOutputStream;",
            "import java.io.PrintStream;");



    @Override
    protected MathCoCoChecker getChecker() {
        return MathCocos.createChecker();
    }
/*
    public static Class loadGeneratedClass(String name) throws Exception {

        File classFile = new File("./");
        URLClassLoader urlClassLoader = URLClassLoader.newInstance(new URL[]{

                new URL("file://" + GENERATEDCLASSTARGETPATH)

        }, URLClassLoader.getSystemClassLoader());

        urlClassLoader.clearAssertionStatus();

        Class classInstance = urlClassLoader.loadClass(BASEPACKAGE + "." + name);

        urlClassLoader.close();

        return classInstance;
    }

    public static Executable getInstanceOfGeneratedClass(String name) throws Exception {
        return (Executable) loadGeneratedClass(name).newInstance();
    }

    /**
     * Compiles the generated java file which is specified by name and returns whether compilation was successful or not.
     * @param name The name of the generated class which should be compiled
     * @return Whether compilation was successful or not
     */ /*
    public boolean compileGeneratedClass(String name) {
        try {
            JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
            createCompilerResultPathIfNotExistent();

            StandardJavaFileManager fileManager = compiler.getStandardFileManager(null, null, null);
            //set class out path
            fileManager.setLocation(StandardLocation.CLASS_OUTPUT, Arrays.asList(new File(GENERATEDCLASSTARGETPATH)));

            File fileToCompile = new File(GENERATEDTARGETPATH+ name + ".java");
            return compiler.getTask(null, fileManager, null, null, null, fileManager.getJavaFileObjects(fileToCompile)).call();


        } catch (Exception ex) {
            ex.printStackTrace();
            Log.info("Compilation Failed", this.getClass().getSimpleName());
        }
        return false;
    }

    private void createCompilerResultPathIfNotExistent() {
        File compilerResultPath = new File(GENERATEDCLASSTARGETPATH);
        if (!compilerResultPath.exists()) {
            compilerResultPath.mkdirs();
        }
    }*/
}
