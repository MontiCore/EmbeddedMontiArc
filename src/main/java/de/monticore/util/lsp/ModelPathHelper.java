package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ModelPathHelper {

    private ModelPathHelper(){
    }

    public static Optional<Path> getPathRelativeTo(Path basePath, Path fullPath){
        if(basePath.getNameCount() > fullPath.getNameCount()){
            return Optional.empty();
        }

        boolean valid = true;
        for(int i = 0; i < basePath.getNameCount(); i++){
            if(!basePath.getName(i).equals(fullPath.getName(i))){
                valid = false;
                break;
            }
        }

        if(valid){
            return Optional.of(fullPath.subpath(basePath.getNameCount(), fullPath.getNameCount()));
        }else{
            return Optional.empty();
        }

    }

    public static Optional<Path> getBasePath(Path fullPath, String packageString) {
        List<String> packageList = Arrays.asList(packageString.split("\\."));
        return getBasePath(fullPath, packageList);
    }


    public static Optional<Path> getBasePath(Path fullPath, List<String> packageList){
        List<String> packageParts = new ArrayList<>(packageList);
        Collections.reverse(packageParts);
        boolean isValid = true;
        for (int i = 0; i < packageParts.size(); i++) {
            int index = fullPath.getNameCount() - 2 - i;
            String packPart = packageParts.get(i);
            String pathPart = fullPath.getName(index).toString();
            Log.debug("Comparing: " + packPart + ", " + pathPart, "default");
            if (!(index >= 0 && packPart.equals(pathPart))) {
                isValid = false;
                Log.error("Path parts not equal: " + packPart + ", " + pathPart);
                break;
            }
        }
        if(isValid) {
            Path modelPath = fullPath.subpath(0, fullPath.getNameCount() - packageParts.size() - 1);
            modelPath = Paths.get(fullPath.getRoot().toString(), modelPath.toString());
            return Optional.of(modelPath);
        }else{
            return Optional.empty();
        }
    }


    public static Path pathFromUriString(String uri) throws URISyntaxException {
        Log.debug("URI: " + uri, "URIs");
        String prefix = "file://";
        String cleanUri = uri.startsWith(prefix) ? uri.substring(prefix.length()) : uri;
        if(uri.contains("%3a") || uri.contains("%3A")) {
            cleanUri = cleanUri.replace("%3A", ":").replace("%3a", ":");
            if(cleanUri.startsWith("\\\\")){
                cleanUri = cleanUri.substring(2);
            }
            if(cleanUri.startsWith("/")){
                cleanUri = cleanUri.substring(1);
            }
            String [] parts = cleanUri.split(":");
            if(parts.length != 2){
                Log.error("Can not get Path from URI: " + uri);
                return null;
            }

            cleanUri = parts[0].toUpperCase() + ":" + parts[1];
        }
        Log.debug("Cleaned URI: " + cleanUri, "URIs");
        return Paths.get(cleanUri);
    }

    public static String encodePathStringToUri(String sourcePath) {
        String fileResourcePrefix = "file://";
        String res = sourcePath;

        if(res.startsWith(fileResourcePrefix)){
            res = res.substring(fileResourcePrefix.length());
        }



        //Convert drive letter identifier to lower case
        if(res.contains(":")){
            String[] parts = res.split(":");
            String[] tail = Arrays.copyOfRange(parts,1,parts.length);
            res = parts[0].toLowerCase() + ":" + String.join(":", tail);
        }

        res = res
                .replace("\\","/")
                .replace(":","%3a");

        if(!res.startsWith("/")){
            res = "/" + res;
        }

        return fileResourcePrefix + res;

    }
}
