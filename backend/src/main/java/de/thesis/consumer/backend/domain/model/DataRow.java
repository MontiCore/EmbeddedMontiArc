package de.thesis.consumer.backend.domain.model;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@Setter
public class DataRow {
	private Long id;
	private String dIdCIdDay;
	private double longitude;
	private double latitude;
	private LocalDateTime gpsTime;
	private double heading;
	private int speed;
	private int odometer;
	private int totalFuelUsed;
	private boolean validPosition;
	private LocalDateTime timestamp;
	private Dataset dataset;
}
