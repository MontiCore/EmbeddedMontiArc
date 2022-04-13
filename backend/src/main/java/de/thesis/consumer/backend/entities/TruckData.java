package de.thesis.consumer.backend.entities;

import lombok.Data;

import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.Id;
import java.time.LocalDateTime;

@Entity
@Data
public class TruckData {
	@Id
	@GeneratedValue
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
}
