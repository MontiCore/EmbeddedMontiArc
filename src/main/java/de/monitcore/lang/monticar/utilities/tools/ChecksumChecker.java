/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities.tools;

import javax.xml.bind.DatatypeConverter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.List;
import java.util.Map;



public class ChecksumChecker {

    protected static MessageDigest messageDigestInstance;
    protected static MessageDigest md5;

    static {
        try {
            messageDigestInstance = MessageDigest.getInstance("SHA-256");
            md5 = MessageDigest.getInstance("MD5");
        } catch (NoSuchAlgorithmException e) {
            e.printStackTrace();
        }
    }

    public static String getChecksumForFileSHA256(String filePath) throws IOException {

        Path wiki_path = Paths.get(filePath);

        List<String> lines = Files.readAllLines(wiki_path);
        StringBuilder sb = new StringBuilder();
        for (String line : lines) {
            sb.append(line);
        }
        String content = sb.toString();
        content = content.replace("\r", "");
        content = content.replace("\n", "");


        messageDigestInstance.reset();
//        messageDigestInstance.update(Files.readAllBytes(Paths.get(filePath)));
        messageDigestInstance.update(content.getBytes());


        byte[] digest = messageDigestInstance.digest();
        return DatatypeConverter.printHexBinary(digest).toUpperCase();
    }



    public static String getChecksumForFileMD5(String filePath) throws IOException {
        Path wiki_path = Paths.get(filePath);

        md5.reset();
        md5.update(Files.readAllBytes(wiki_path));

        byte[] digest = md5.digest();
        return DatatypeConverter.printHexBinary(digest).toUpperCase();
/*
        List<String> lines = Files.readAllLines(wiki_path);
        StringBuilder sb = new StringBuilder();
        for (String line : lines) {
            sb.append(line);
        }
        String content = sb.toString();
        content = content.replace("\r", "");
        content = content.replace("\n", "");

        return getChecksumForStringMD5(content);*/
    }

    public static String getChecksumForStringMD5(String s){
        md5.reset();
        md5.update(s.getBytes());

        byte[] digest = md5.digest();
        return DatatypeConverter.printHexBinary(digest).toUpperCase();
    }

    public static boolean checkFile(String filePath, String checksum) throws IOException {
        return getChecksumForFileSHA256(filePath).equalsIgnoreCase(checksum);
    }

    public static void checkFilesWithAssert(Map<String,String> fileChecksums) throws IOException {
        for (Map.Entry<String,String> entry:fileChecksums.entrySet()) {
            String checksum = getChecksumForFileSHA256(entry.getKey());
/*
            TestCase.assertTrue(
                    "File "+entry.getKey()+" is not correct! (expected: "+entry.getValue()+", value: "+checksum+")",
                    checksum.equalsIgnoreCase(entry.getValue())
            );
            */
        }
    }
}
