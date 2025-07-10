package entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@Data
@AllArgsConstructor
@NoArgsConstructor
public class Dataset {
	private UUID id;
	private UUID offerId;
	private Metadata metadata;
	private List<DataRow> data;
	private LocalDateTime boughtAt;

	public int getNumberOfTrucks() {
		return data.size();
	}

	public int getRestingTrucks() {
		int count = 0;

		for (DataRow datarow : data) {
			if (datarow.getSpeed() == 0) {
				count++;
			}
		}

		return count;
	}

	public int getDrivingTrucks() {
		int count = 0;

		for (DataRow datarow : data) {
			if (datarow.getSpeed() > 0) {
				count++;
			}
		}

		return count;
	}

	public long getFuelConsumption() {
		long totalFuel = 0;
		for (DataRow datarow : data) {
			totalFuel += datarow.getTotalFuelUsed();
		}

		return totalFuel / data.size();
	}
}
