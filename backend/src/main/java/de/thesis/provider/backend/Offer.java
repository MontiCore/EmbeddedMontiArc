package de.thesis.provider.backend;

import de.thesis.provider.backend.csv.DataRow;
import de.thesis.provider.backend.dto.CreatePolicyDto;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDate;
import java.util.List;
import java.util.UUID;

@Getter
@Setter
public class Offer {
	private UUID id;
	private String title;
	private String provider;
	private String description;
	private double price;
	private Policy policy;
	private LocalDate expiresOn;
	private List<DataRow> data;
}
