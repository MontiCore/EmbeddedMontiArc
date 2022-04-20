package de.thesis.consumer.backend.persistence.entity;

import com.fasterxml.jackson.annotation.JsonIgnore;
import lombok.Data;

import javax.persistence.*;
import java.time.LocalDateTime;

@Entity
@Data
public class DataRowEntity {
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
	@ManyToOne
	@JoinColumn(name = "dataset")
	@JsonIgnore
	private DatasetEntity dataset;
}
