package dto;

import entity.DataRow;
import entity.Metadata;
import lombok.Value;

import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@Value
public class DatasetViewDto {
	UUID id;
	Metadata metadata;
	List<DataRow> data;
	LocalDateTime boughtAt;
	int numberOfTrucks;
	int restingTrucks;
	int drivingTrucks;
	long avgFuelConsumption;
	double insuranceFee;
}
