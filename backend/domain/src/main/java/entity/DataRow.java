package entity;

import lombok.Data;
import lombok.Value;

import java.time.LocalDateTime;

@Data
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
	LocalDateTime timestamp;
}
