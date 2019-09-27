/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * Abstract Builder class for a ModelicaPhysicalVehicle to avoid complex constructors
 */
public class ModelicaPhysicalVehicleBuilder extends PhysicalVehicleBuilder {

    /**
     * Constructor
     */
    public ModelicaPhysicalVehicleBuilder(){
        // Class has no uninitialized fields
    }

	@Override
	PhysicalVehicle createPhysicalVehicle() {
		return new ModelicaPhysicalVehicle();
    }
    
    

	@Override
	RealVector calculateAngularVelocity(RealVector angularVelocity) {
		// Get rotation
        RealMatrix rotation;
        if(this.rotation.isPresent()){
            rotation = ModelicaPhysicalVehicle.coordinateRotation.multiply(new BlockRealMatrix(this.rotation.get().getMatrix()));
        }else{
            rotation = ModelicaPhysicalVehicle.coordinateRotation.copy();
        }
        // Compute angular velocity in local coordinates
        return rotation.transpose().operate(this.angularVelocity.get());
	}

	@Override
	void setVelocity(PhysicalVehicle physicalVehicle, RealVector velocity) {
        // Get rotation
        RealMatrix rotation;
        if(this.rotation.isPresent()){
            rotation = ModelicaPhysicalVehicle.coordinateRotation.multiply(new BlockRealMatrix(this.rotation.get().getMatrix()));
        }else{
            rotation = ModelicaPhysicalVehicle.coordinateRotation.copy();
        }
        // Compute velocity in local coordinates
        RealVector localVelocity = rotation.transpose().operate(this.velocity.get());
        // Set initial velocity values
        ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("v_x_0", localVelocity.getEntry(0));
        ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("v_y_0", localVelocity.getEntry(1));
        // Get wheelRadius
        double wheelRadius;
        if(this.wheelRadius.isPresent()){
            wheelRadius = this.wheelRadius.get();
        }else {
            VehicleDynamicsModel model = new VehicleDynamicsModel();
            model.initialise();
            wheelRadius = model.getValue("r_nom");
        }
        localVelocity.setEntry(0, localVelocity.getEntry(0) / wheelRadius);
        // Set initial wheel rotation rates
        ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_1_0", localVelocity.getEntry(0));
        ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_2_0", localVelocity.getEntry(0));
        ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_3_0", localVelocity.getEntry(0));
        ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_4_0", localVelocity.getEntry(0));
	}
}
