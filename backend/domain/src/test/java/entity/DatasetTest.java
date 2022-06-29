package entity;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;

class DatasetTest {

	@Test
	void shouldReturnThreeTrucks() {
		Dataset dataset = new Dataset();
		dataset.setData(List.of(new DataRow(), new DataRow(), new DataRow()));

		assertThat(dataset.getNumberOfTrucks()).isEqualTo(3);
	}

	@Test
	void shouldReturnTwoRestingTrucks() {
		Dataset dataset = new Dataset();
		DataRow fastTruck = new DataRow();
		fastTruck.setSpeed(100);
		DataRow restingTruck1 = new DataRow();
		restingTruck1.setSpeed(0);
		DataRow slowTruck = new DataRow();
		slowTruck.setSpeed(13);
		DataRow restingTruck2 = new DataRow();
		restingTruck2.setSpeed(0);

		dataset.setData(List.of(fastTruck, restingTruck1, restingTruck2, slowTruck));

		assertThat(dataset.getRestingTrucks()).isEqualTo(2);
	}

	@Test
	void shouldReturnOneDrivingTruck() {
		Dataset dataset = new Dataset();
		DataRow fastTruck = new DataRow();
		fastTruck.setSpeed(100);
		DataRow restingTruck1 = new DataRow();
		restingTruck1.setSpeed(0);
		DataRow slowTruck = new DataRow();
		slowTruck.setSpeed(13);
		DataRow restingTruck2 = new DataRow();
		restingTruck2.setSpeed(0);

		dataset.setData(List.of(fastTruck, restingTruck1));

		assertThat(dataset.getDrivingTrucks()).isEqualTo(1);
	}

	@Test
	void shouldReturnFuelConsumptionOf50Liters() {
		Dataset dataset = new Dataset();
		DataRow truck1 = new DataRow();
		truck1.setTotalFuelUsed(100);
		DataRow truck2 = new DataRow();
		truck2.setTotalFuelUsed(25);
		DataRow truck3 = new DataRow();
		truck3.setTotalFuelUsed(25);

		dataset.setData(List.of(truck1, truck2, truck3));

		assertThat(dataset.getFuelConsumption()).isEqualTo(50);
	}
}
