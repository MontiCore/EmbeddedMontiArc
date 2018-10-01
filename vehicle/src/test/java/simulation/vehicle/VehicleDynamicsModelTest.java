package simulation.vehicle;

import org.junit.Test;

/**
 * Class that tests the VehicleDynamicsModel class
 */
public class VehicleDynamicsModelTest {
    @Test(expected = IllegalStateException.class)
    public void initializeFail(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialize();
        model.initialize();
    }

    @Test
    public void doStepNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialize();
        model.doStep(0.033);
    }

    @Test(expected = IllegalStateException.class)
    public void doStepFail(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.doStep(0.033);
    }

    @Test
    public void setParameterNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.setParameter("m", 1750);
    }

    @Test(expected = IllegalStateException.class)
    public void setParameterFail(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialize();
        model.setParameter("m", 1750);
    }

    @Test
    public void setInputNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialize();
        model.setInput("delta_1", 0.3);
    }

    @Test(expected = IllegalStateException.class)
    public void setInputFail(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.setInput("delta_1", 0.3);
    }

    @Test
    public void getValueNormal(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.initialize();
        model.getValue("m");
    }

    @Test(expected = IllegalStateException.class)
    public void getValueFail(){
        VehicleDynamicsModel model = new VehicleDynamicsModel();
        model.getValue("m");
    }
}
