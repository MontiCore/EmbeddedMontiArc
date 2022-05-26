package entity;

import lombok.Data;

import java.time.LocalDate;

@Data
public class Metadata {
	Long id;
	String title;
	String provider;
	String description;
	double price;
	LocalDate expiresOn;
	String loggingUrl;
	Policy policy;
}
