/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth_aachen.se.montisim.simulators.basic_simulator.gui;

import de.rwth_aachen.se.montisim.simulators.basic_simulator.filesystem.FileSystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.LinkedHashSet;

public class Category implements FileSystem.DirectoryWatcher {


    public enum CategoryType {
        NONE("none", "", false, "", false),
        AUTOPILOTS("autopilots", "Autopilots", true, "", true),
        SCENARIOS("scenarios", "Scenarios", true, "json", false),
        MAPS("maps", "Maps", true, "osm", false),
        RESULTS("results", "Results", true, "json", false),
        SIMULATIONS("simulations", "Simulations", false, "", false);
        public final String id;
        public final String name;
        public final boolean directory;
        public final String extension;
        public final boolean show_extensions;
        CategoryType(String id, String name, boolean directory, String extension, boolean show_extensions){
            this.name = name;
            this.id = id;
            this.directory = directory;
            this.extension = extension;
            this.show_extensions = show_extensions;
        }
    }

    public static class Elem {
        public final String id; //Path in most cases
        public final String name; //Name in the tree
        Elem(String id, String name){
            this.id = id;
            this.name = name;
        }
        public String toString(){
            return name;
        }
    }


    LinkedHashSet<String> elems = new LinkedHashSet<>();
    final CategoryType type;
    SimVis vis;
    Browser browser;

    public Category(CategoryType type, SimVis vis, Browser browser, FileSystem file_system) throws IOException {
        this.type = type;
        this.vis = vis;
        this.browser = browser;

        browser.register_category(this);

        if (type.directory){
            file_system.register_directory(this, type.id, type.extension); //Initializes the list => Tree must be linked
        }
    }

    public SimVis getVis(){
        return vis;
    }

    public CategoryType getType(){
        return type;
    }

    @Override
    public void add_item(String path, String name) {
        Elem elem = new Elem(path, name);
        elems.add(path);
        browser.add_item(this, elem);
    }

    @Override
    public void remove_item(String path){
        elems.remove(path);
        browser.remove_item(this, path);
    }
}
