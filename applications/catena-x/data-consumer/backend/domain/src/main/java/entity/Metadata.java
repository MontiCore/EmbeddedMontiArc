package entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class Metadata {
	private int id;
	private String title;
	private String provider;
	private String description;
	private double price;
	private String loggingUrl;
	private Policy policy;
}
