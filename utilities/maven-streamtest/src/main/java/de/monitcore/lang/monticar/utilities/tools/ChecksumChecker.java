/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities.tools;

import org.apache.commons.codec.digest.DigestUtils;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.List;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.IOUtils;

public class ChecksumChecker {
    // From https://stackoverflow.com/questions/9655181/how-to-convert-a-byte-array-to-a-hex-string-in-java
    private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();
    public static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for (int j = 0; j < bytes.length; j++) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = HEX_ARRAY[v >>> 4];
            hexChars[j * 2 + 1] = HEX_ARRAY[v & 0x0F];
        }
        return new String(hexChars);
    }

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
        return bytesToHex(digest);
    }



    public static String getChecksumForFileMD5(String filePath) throws IOException {
        Path wiki_path = Paths.get(filePath);

        md5.reset();
        md5.update(Files.readAllBytes(wiki_path));

        byte[] digest = md5.digest();
        return bytesToHex(digest);
    }

    public static String getChecksumForStringMD5(String s){
        md5.reset();
        md5.update(s.getBytes());

        byte[] digest = md5.digest();
        return bytesToHex(digest);
    }

    public static boolean checkFile(String filePath, String checksum) throws IOException {
        return getChecksumForFileSHA256(filePath).equalsIgnoreCase(checksum);
    }

    public static String getChecksumForFileSHA1(String s){
        String hash = "";
        Log.info("Start hashing of file " + s, ChecksumChecker.class.getSimpleName());
        try(FileInputStream inputStream = new FileInputStream(s)) {
            hash = DigestUtils.sha1Hex(inputStream);
        } catch (IOException e){
            e.printStackTrace();
            Log.info("Exception during hashing the file " + s, ChecksumChecker.class.getSimpleName());
        }
        Log.info("Finished hashing of file " + s, ChecksumChecker.class.getSimpleName());
        return hash;
    }

}
