package de.thesis.provider.backend.entities;

import lombok.Data;

@Data
public class Metadata {
	private String title;
	private String provider;
	private String description;
	private double price;
	private String loggingUrl;
	private Policy policy;
}
