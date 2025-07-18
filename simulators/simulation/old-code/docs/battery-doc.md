Power Train(S19) - Battery Group
====================


**Introduction**
----------------
----------
This introduction is divided into two main sections Battery Charging and Battery Discharging, which we thought it was
the best way to display our work, as they are the main scenarios in our case. For each of the sections a diagram is
displayed, in order to ease the understanding behind our ideas and the code. Also, there is provided a brief description
of the important functionalities.
**Battery** is a class on which we use as our main class and provides the functionalities of the battery, so our **
BatteryInterface** can distribute it to the rest of the classes.
**DummyBattery** is a simple class, which is chosen to display our Battery Model, where we set our basic functions of
the battery. This instance is part of the Battery class and we use it in our both scenarios.
On the **Vehicle** class are added some functionalities, which makes possible for our code to run smoothly.

**Battery Charging**
----------------
----------

![enter image description here](https://lh3.googleusercontent.com/-x75VNqVuxlE/XW4eMGpJMYI/AAAAAAAABtQ/dnrAe5h-Ey8znbCnPw6icE_XYwIYkgG8wCLcBGAs/s0/Battery_Charging.PNG "Battery_Charging")

The scenario for charging the battery of the Car consists of three main elements:

- the Battery itself,
- Charging Station and the Cable, which is part of the Charging Station.

In order for the charging process to start, the *ChargingStationConnectionStatus* has to be set to true, which means the
Car is connected to the Charging Station. Setting the value of this ChargingStationConnectionStatus is decided to be set
by the ChargingStation, as it is the object that detects the nearby cars. When the car leaves, the proximity of a
ChargingStation object, that ChargingStation has to set ChargingStationConnectionStatus to false. Check of whether
battery should be charged is done in the Vehicle class, using this ChargingStationConnectionStatus. Also, for ease of
implementation and testing, parameters like *ChargingStationVoltage* and *ChargingStationAmpere* are set to some default
values, respectively 100 and 1. After merging the implementations, those parameters would be provided by a
ChargingStation. These two parameters are the only ones that are sent to the Battery from the side of Charging Station.

In addition, there is also a cable Resistance, but in our case we are not taking it into account, and just putting it
some default value through the *setChargingCableResistance* function at *DummyBattery* class, this means we choose to
have a linear working Resistance.

For a better estimation we take into account the *local_deltaT* parameter. This parameter is the Simulation Delta Time
and is taken from the Bus Controller inside the Vehicle object. After all the parameters have been provided and the
connection with the Charging Station is established the charging process starts. The power added to the battery is
directly depended on these three parameters VoltageChargingStation, AmpereChargingStation and local_deltaT.

During the process the Battery can provide parameters like *timeToCharge* and *currentPercentage* to the car.

**Battery Discharging**
-------------------
----------
![enter image description here](https://lh3.googleusercontent.com/tHrJQ-2TbFFBq5ztSHW8ghO_-O11LYt36tlsLqjky4_az4ehsP13qPJ39xRxboxc7tr7JScGK92a=s0 "Battery_Discharging")

For the discharging process, we only have two main elements: the Engine and the Battery. Both of them communicate with
each other in a circular manner. The Engine is sending information to the Battery and Battery responds by whether the
operation is successful or not.

In Vehicle class, there exists a function called exchangeDataWithController, where an instance of Vehicle sends
information to and receives information from various components of itself. We decided it is best to charge & discharge
the battery in this function.

When discharge function of Battery is invoked, Battery asks for some information from Engine to calculate energy needed
to provide the said discharge. Those information are:

- {mass, velocity} for MassPointPhysicalVehicle
- {throttle, gear} for ModelicaPhysicalVehicle

Using the said information, Battery calculates the necessary energy to provide energy, in the following way:

- for MassPointPhysicalVehicle: subtract the newly calculated kinetic energy from the previous one
- For ModelicaPhysicalVehicle: calculate energy consumption via multiplying throttle and gear in a determined way (
  albeit, arbitrarily)

After the calculation of necessary energy, Battery converts those energy values into voltage and ampere values to send
those information to DummyBattery (in place of Modelica model). Conversion of energy into (voltage, ampere) pair is done
arbitrarily. We assumed a linear model, and set the ampere value to 1.

- defaultAmpere = 1.0
- consumptionAmpere = defaultAmpere
- consumptionVoltage = energyConsumption / consumptionAmpere

Those consumptionVoltage and consumptionAmpere values are passed to a DummyBattery instance, which is contained inside a
Battery instance. DummyBattery, if possible, would act accordingly and discharge (using energy calculation via formula
Voltage*Ampere*Time, time being the delta time, based on the simulation specification) and would not return any value.
In case where battery is below a critical level (which is determined to be 10%), DummyBattery would throw an exception,
indicating that DummyBattery cannot discharge. Battery would catch this exception, and this means we cannot accelerate
anymore our Vehicle. Battery would also throw an exception to the Vehicle, setting a flag of Vehicle that is called
BatteryProblem to true. Based on this flag, we set either motor or throttle to zero, based on the type of the car (Mass
or Modelica). Otherwise, we decrease the corresponding power from the Battery and nothing is returned to the Car.

When the calculations are done and the corresponding power of the Battery is consumed, we sent information like *
currentPercentage* and *timeToCharge* to the engine and allow it to consume or not more power.

**Authors**
-------------------
----------

- Anxhela Hyseni (391289) anxhela.hyseni1996@gmail.com, MSc Media Informatics
- Ulfet Cetin (391819) ulfet.rwth@gmail.com, MSc Software Systems Engineering
