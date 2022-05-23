package entity;

import lombok.Value;

import java.time.LocalDateTime;

@Value
public class DataRow {
	Long id;
	String dayID;
	double longitude;
	double latitude;
	LocalDateTime gpsTime;
	double heading;
	int speed;
	int odometer;
	int totalFuelUsed;
	boolean validPosition;
	LocalDateTime timestamp;
}
