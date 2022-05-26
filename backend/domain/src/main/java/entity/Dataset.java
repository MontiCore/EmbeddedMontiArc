package entity;

import lombok.Data;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@Data
public class Dataset {
	UUID id;
	String title;
	String provider;
	String description;
	double price;
	Policy policy;
	LocalDateTime boughtAt;
	LocalDate expiresOn;
	List<DataRow> data;
	String loggingUrl;
}
