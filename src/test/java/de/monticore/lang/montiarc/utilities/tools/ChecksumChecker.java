package de.monticore.lang.montiarc.utilities.tools;

import javax.xml.bind.DatatypeConverter;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.List;
import java.util.Map;

import static junit.framework.TestCase.assertTrue;

public class ChecksumChecker {

    protected static MessageDigest messageDigestInstance;

    static {
        try {
            messageDigestInstance = MessageDigest.getInstance("SHA-256");
        } catch (NoSuchAlgorithmException e) {
            e.printStackTrace();
        }
    }

    public static String getChecksumForFile(String filePath) throws IOException {

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

    public static boolean checkFile(String filePath, String checksum) throws IOException {
        return getChecksumForFile(filePath).equalsIgnoreCase(checksum);
    }

    public static void checkFilesWithAssert(Map<String,String> fileChecksums) throws IOException {
        for (Map.Entry<String,String> entry:fileChecksums.entrySet()) {
            String checksum = getChecksumForFile(entry.getKey());

            assertTrue(
                    "File "+entry.getKey()+" is not correct! (expected: "+entry.getValue()+", value: "+checksum+")",
                    checksum.equalsIgnoreCase(entry.getValue())
            );
        }
    }
}
