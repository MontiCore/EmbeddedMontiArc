/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.Bus;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.FunctionBlockInterface;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealVector;
import simulation.EESimulator.EESimulator;

import java.util.Optional;

/**
 * Abstract Builder class for a PhysicalVehicle to avoid complex constructors
 */
public abstract class PhysicalVehicleBuilder {

	protected Optional<Vehicle> vehicle = Optional.empty();
	
    protected Optional<RealVector> position = Optional.empty();
    protected Optional<Rotation> rotation = Optional.empty();
    protected Optional<RealVector> velocity = Optional.empty();
    protected Optional<RealVector> angularVelocity = Optional.empty();

    protected Optional<Double> mass = Optional.empty();

    protected Optional<Double> width = Optional.empty();
    protected Optional<Double> length = Optional.empty();
    protected Optional<Double> height = Optional.empty();

    protected Optional<Double> wheelRadius = Optional.empty();
    protected Optional<Double> wheelDistLeftRightFrontSide = Optional.empty();
    protected Optional<Double> wheelDistLeftRightBackSide = Optional.empty();
    protected Optional<Double> wheelDistToFront = Optional.empty();
    protected Optional<Double> wheelDistToBack = Optional.empty();

    //TODO: Add actuators to the build process and to the JSON serialisation

    protected Optional<Optional<FunctionBlockInterface>> navigation = Optional.empty();

    /**
     * Constructor
     */
    public PhysicalVehicleBuilder() {

    }

    /**
     * Function that returns a PhysicalVehicle with the attributes currently stored in the builder
     *
     * @return PhysicalVehicle that was built with the builder
     */
    public PhysicalVehicle buildPhysicalVehicle() {
    	PhysicalVehicle physicalVehicle = this.createPhysicalVehicle();
        
    	if(this.velocity.isPresent()) {
    		this.setVelocity(physicalVehicle, velocity.get());
    	}
    	if(this.angularVelocity.isPresent()) {
    		physicalVehicle.setAngularVelocity(this.calculateAngularVelocity(angularVelocity.get()));
    	}
    	
    	this.vehicle.ifPresent(physicalVehicle::setVehicle);
    	
    	this.mass.ifPresent(physicalVehicle::setMass);

        this.width.ifPresent(physicalVehicle::setWidth);
        this.length.ifPresent(physicalVehicle::setLength);
        this.height.ifPresent(physicalVehicle::setHeight);
        
    	this.wheelRadius.ifPresent(physicalVehicle::setWheelRadius);
        this.wheelDistLeftRightFrontSide.ifPresent(physicalVehicle::setWheelDistLeftRightFrontSide);
        this.wheelDistLeftRightBackSide.ifPresent(physicalVehicle::setWheelDistLeftRightBackSide);
        this.wheelDistToFront.ifPresent(physicalVehicle::setWheelDistToFront);
        this.wheelDistToBack.ifPresent(physicalVehicle::setWheelDistToBack);
        
        //create vehicle that pyhsicalVehicle belongs to
        physicalVehicle.setVehicle(new Vehicle(physicalVehicle));
        physicalVehicle.initializeActuators();
        
        physicalVehicle.initPhysics();
        
        this.position.ifPresent(physicalVehicle::setPosition);
        this.rotation.ifPresent(rotation -> physicalVehicle.setRotation(new BlockRealMatrix(rotation.getMatrix())));
    	
       
        
        return physicalVehicle;
    }
    
    /**
     * Function that returns a uninitialized PhysicalVehicle (freshly created)
     *
     * @return PhysicalVehicle that is uninitialized
     */
    abstract PhysicalVehicle createPhysicalVehicle();
    
    abstract RealVector calculateAngularVelocity(RealVector angularVelocity);
    
    abstract void setVelocity(PhysicalVehicle vehicle, RealVector velocity);

    public PhysicalVehicleBuilder setPosition(RealVector position){
        this.position = Optional.of(position);
        return this;
    }

    public PhysicalVehicleBuilder setRotation(Rotation rotation){
        this.rotation = Optional.of(rotation);
        return this;
    }

    public  PhysicalVehicleBuilder setVelocity(RealVector velocity){
        this.velocity = Optional.of(velocity);
        return this;
    }

    public  PhysicalVehicleBuilder setAngularVelocity(RealVector angularVelocity){
        this.angularVelocity = Optional.of(angularVelocity);
        return this;
    }

    public  PhysicalVehicleBuilder setMass(double mass){
        this.mass = Optional.of(mass);
        return this;
    }

    public  PhysicalVehicleBuilder setWheelRadius(double wheelRadius){
        this.wheelRadius = Optional.of(wheelRadius);
        return this;
    }

    public  PhysicalVehicleBuilder setWidth(double width){
        this.width = Optional.of(width);
        return this;
    }

    public  PhysicalVehicleBuilder setLength(double length){
        this.length = Optional.of(length);
        return this;
    }

    public  PhysicalVehicleBuilder setHeight(double height){
        this.height = Optional.of(height);
        return this;
    }

    public  PhysicalVehicleBuilder setWheelDistLeftRightFrontSide(double wheelDistLeftRightFrontSide){
        this.wheelDistLeftRightFrontSide = Optional.of(wheelDistLeftRightFrontSide);
        return this;
    }

    public  PhysicalVehicleBuilder setWheelDistLeftRightBackSide(double wheelDistLeftRightBackSide){
        this.wheelDistLeftRightBackSide = Optional.of(wheelDistLeftRightBackSide);
        return this;
    }

    public  PhysicalVehicleBuilder setWheelDistToFront(double wheelDistToFront){
        this.wheelDistToFront = Optional.of(wheelDistToFront);
        return this;
    }

    public  PhysicalVehicleBuilder setWheelDistToBack(double wheelDistToBack){
        this.wheelDistToBack = Optional.of(wheelDistToBack);
        return this;
    }

    public PhysicalVehicleBuilder setNavigation(Optional<FunctionBlockInterface> navigation) {
        this.navigation = Optional.of(navigation);
        return this;
    }
}
