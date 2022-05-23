package entity;

import lombok.Value;

import java.time.LocalDate;
import java.util.List;
import java.util.UUID;

@Value
public class Offer {
	UUID id;
	String title;
	String provider;
	String description;
	double price;
	Policy policy;
	LocalDate expiresOn;
	List<DataRow> data;
	String loggingUrl;
}
