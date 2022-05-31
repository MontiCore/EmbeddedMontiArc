package entity;

import lombok.Data;
import lombok.Value;

import java.time.LocalDateTime;

@Data
public class DataRow {
	private int id;
	private String dayID;
	private double longitude;
	private double latitude;
	private LocalDateTime gpsTime;
	private double heading;
	private int speed;
	private int odometer;
	private int totalFuelUsed;
	private LocalDateTime timestamp;
}
