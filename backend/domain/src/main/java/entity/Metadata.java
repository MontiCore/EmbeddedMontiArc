package entity;

import lombok.Data;

import java.time.LocalDate;

@Data
public class Metadata {
	private Long id;
	private String title;
	private String provider;
	private String description;
	private double price;
	private LocalDate expiresOn;
	private String loggingUrl;
	private Policy policy;
}
