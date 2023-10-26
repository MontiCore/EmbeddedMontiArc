package de.monticore.lang.monticar.utilities.configcheck;

import java.nio.charset.StandardCharsets;
import java.util.Base64;

public class Encoder {
    public static String encode(String input) {
        byte[] encodedBytes = Base64.getEncoder().encode(input.getBytes());
        return new String(encodedBytes, StandardCharsets.UTF_8).replaceAll("[^a-zA-Z0-9]", "");
    }

    public static String decode(String encodedString) {
        // Add padding if necessary to ensure it's a valid Base64 string
        int padding = (4 - (encodedString.length() % 4)) % 4;
        StringBuilder paddedEncodedString = new StringBuilder(encodedString);
        for (int i = 0; i < padding; i++) {
            paddedEncodedString.append("=");
        }
        byte[] originalBytes = Base64.getDecoder().decode(paddedEncodedString.toString());
        return new String(originalBytes, StandardCharsets.UTF_8);
    }
}