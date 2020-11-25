package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import de.monticore.lang.monticar.generator.FileContent;
import de.rwth.montisim.commons.utils.json.SerializationException;

public class DDCCommunication {
    static final String BASE_SPACES = "        ";

    final DynamicInterfaceGenerator gen;

    FileBuilder b = new FileBuilder();

    
    public DDCCommunication(DynamicInterfaceGenerator gen) {
        this.gen = gen;
        throw new IllegalArgumentException("DDCCommunication not implemented yet.");
    }
    
    public List<FileContent> generate() throws SerializationException {
        List<FileContent> files = new ArrayList<>();
        
        return files;
    }
    
	public void getSources(HashSet<String> sources) {
        sources.add("ddc_mode.cpp");
    }
    public void getLibs(HashSet<String> libs) {
        libs.add("ddc_load");
    }
}
