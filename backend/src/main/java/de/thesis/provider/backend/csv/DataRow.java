package de.thesis.provider.backend.csv;

import com.opencsv.bean.CsvBindByName;
import com.opencsv.bean.CsvCustomBindByName;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@Setter
public class DataRow {
	@CsvBindByName(column = "id")
	private int id;
	@CsvBindByName(column = "dayID")
	private String dayID;
	@CsvBindByName(column = "longitude")
	private double longitude;
	@CsvBindByName(column = "latitude")
	private double latitude;
	@CsvCustomBindByName(column = "gpsTime", converter = LocalDateTimeConverter.class)
	private LocalDateTime gpsTime;
	@CsvBindByName(column = "heading")
	private double heading;
	@CsvBindByName(column = "speed")
	private int speed;
	@CsvBindByName(column = "odometer")
	private int odometer;
	@CsvBindByName(column = "totalFuelUsed")
	private int totalFuelUsed;
	@CsvCustomBindByName(column = "timestamp", converter = LocalDateTimeConverter.class)
	private LocalDateTime timestamp;
}
