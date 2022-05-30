package de.thesis.provider.backend;

import lombok.Data;
import org.springframework.beans.factory.annotation.Value;

@Data
public class Metadata {
	String title;
	@Value("${PROVIDER_NAME}")
	String provider;
	String description;
	double price;
	@Value("${PROVIDER_URL}")
	String loggingUrl;
	Policy policy;
}
