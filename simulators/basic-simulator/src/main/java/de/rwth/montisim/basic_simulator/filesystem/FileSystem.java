/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.filesystem;

import java.io.*;
import java.nio.file.*;
import java.util.HashMap;
import java.util.Map;

import static java.nio.file.StandardWatchEventKinds.*;

public class FileSystem {

    public interface DirectoryWatcher {
        void add_item(String path, String name);
        void remove_item(String path);
    }

    public static class DirInfo {
        Path path;
        DirectoryWatcher watcher;
        String extension;
        public DirInfo(Path path, DirectoryWatcher watcher, String extension){
            this.path = path;
            this.watcher = watcher;
            this.extension = extension;
        }
    }

    private final String working_dir;
    private final WatchService watcher;
    private final Map<WatchKey,DirInfo> keys = new HashMap<>();


    /*
        https://howtodoinjava.com/java8/java-8-watchservice-api-tutorial/

     */
    public FileSystem(String working_dir) throws IOException {
        this.working_dir = working_dir;
        this.watcher = FileSystems.getDefault().newWatchService();
    }

    public void register_directory(DirectoryWatcher watcher, String directory_name, String extension) throws IOException {
        Path dir = Paths.get(working_dir + directory_name);
        File dir_file = dir.toFile();
        dir_file.mkdirs();
        WatchKey key = dir.register(this.watcher, ENTRY_CREATE, ENTRY_DELETE, ENTRY_MODIFY);
        keys.put(key, new DirInfo(dir, watcher, extension));


        for (File file : dir_file.listFiles()){
            add_file(watcher, file, extension);
        }
    }

    public void add_file(DirectoryWatcher watcher, File file, String extension) {
        if(file == null) return;
        if (!file.isDirectory()){
            String file_name = file.getName();
            String file_ext = getExtension(file_name);
            if (extension.length() == 0 || extension.equalsIgnoreCase(file_ext)){
                if (extension.length() != 0){
                    file_name = file_name.substring(0, file_name.length() - (file_ext.length() +1));
                }
                watcher.add_item(file.getPath(), file_name);
            }
        }
    }

    public void remove_file(DirectoryWatcher watcher, File file, String extension) {
        if(file == null) return;
        if (!file.isDirectory()){
            watcher.remove_item(file.getPath());
        }
    }

    public void check_updates(){
        for (;;){
            WatchKey key;
            try {
                key = watcher.take();
            } catch (InterruptedException x) {
                return;
            }
            DirInfo target = keys.get(key);
            if (target == null) continue;

            for (WatchEvent<?> event : key.pollEvents()) {
                @SuppressWarnings("rawtypes")
                WatchEvent.Kind kind = event.kind();
                // Context for directory entry event is the file name of entry
                Path name = ((WatchEvent<Path>)event).context();
                Path child = target.path.resolve(name);

                // print out event
                //System.out.format("%s: %s\n", event.kind().name(), child);

                if (kind == ENTRY_CREATE) {
                    add_file(target.watcher, child.toFile(), target.extension);
                } else if (kind == ENTRY_DELETE){
                    remove_file(target.watcher, child.toFile(), target.extension);
                } else if (kind == ENTRY_MODIFY){

                }
            }

            // reset key and remove from set if directory no longer accessible
            boolean valid = key.reset();
            if (!valid) {
                keys.remove(key);

                // all directories are inaccessible
                if (keys.isEmpty()) {
                    break;
                }
            }
        }
    }

    public enum FileType {
        JSON("json"),
        SIM("sim"),
        OSM("osm"),
        UNKNOWN("");
        private String extension;
        FileType(String extension){
            this.extension = extension;
        }
        public String get_extension(){
            return this.extension;
        }
    }

    public static String getExtension(String file){
        int i = file.lastIndexOf('.');
        if (i > 0) {
            return file.substring(i+1);
        }
        return "";
    }

    public static FileType getFileType(String file){
        String extension = getExtension(file);
        if (extension.equalsIgnoreCase(FileType.JSON.get_extension())) return FileType.JSON;
        if (extension.equalsIgnoreCase(FileType.SIM.get_extension())) return FileType.SIM;
        if (extension.equalsIgnoreCase(FileType.OSM.get_extension())) return FileType.OSM;
        return FileType.UNKNOWN;
    }

    public File getPath(String dir, String file){
        return new File(working_dir + dir + "/" + file);
    }

}
