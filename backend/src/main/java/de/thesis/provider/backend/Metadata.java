package de.thesis.provider.backend;

import lombok.Data;
import org.springframework.beans.factory.annotation.Value;

@Data
public class Metadata {
	private String title;
	private String provider;
	private String description;
	private double price;
	private String loggingUrl;
	private Policy policy;
}
