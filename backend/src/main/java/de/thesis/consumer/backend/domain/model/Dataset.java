package de.thesis.consumer.backend.domain.model;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;
import java.util.List;
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
	private LocalDateTime boughtAt;
	private List<DataRow> data;
}
