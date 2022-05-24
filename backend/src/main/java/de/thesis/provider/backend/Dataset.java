package de.thesis.provider.backend;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalDate;
import java.util.UUID;

@Getter
@Setter
public class Dataset {
	private UUID id;
	private String title;
	private String provider;
	private String description;
	private double price;
	private Policy policy;
	private LocalDate expiresOn;
}
