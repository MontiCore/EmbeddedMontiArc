package dto;

import entity.DataRow;
import entity.Policy;
import lombok.Value;

import java.time.LocalDate;
import java.util.List;

@Value
public class AddOfferCommand {
	String title;
	String provider;
	String description;
	double price;
	Policy policy;
	LocalDate expiresOn;
	List<DataRow> data;
	String loggingUrl;
}
