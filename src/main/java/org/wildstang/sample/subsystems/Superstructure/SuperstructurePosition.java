package org.wildstang.sample.subsystems.Superstructure;

public enum SuperstructurePosition {
//first value its arm second value lift position.
    CORAL_STATION_FRONT (9,0, "Coral Station Front"),
    CORAL_STATION_BACK(106,0, "Coral Station Back"),
    ALGAE_PRESTAGED(100,0, "Algae Prestaged"),
    ALGAE_REEF_LOW(35,7, "Algae Pickup Low"),//was 87 4
    ALGAE_REEF_HIGH(35,29, "Algae Pickup High"),// was 87 27
    CORAL_REEF_L1(90,0, "Reef L1"),//arms was 36, lifts all +2
    CORAL_REEF_L2(84,17, "Reef L2"),//80.3, 24.5
    CORAL_REEF_L3(84,39.5, "Reef L3"),//80.5, 45.5
    CORAL_REEF_L4 (80.3,80.5, "Reef L4"),
    ALGAE_NET_FRONT(81,80, "Algae Net Front"),
    ALGAE_NET_BACK(50,70,"Algae Net Back"),
    ALGAE_PROCESSOR_BACK(22,0, "Algae Proc Back"),
    ALGAE_PROCESSOR_FRONT(104,0, "Algae Proc Front"),
    STOWED_AFTER_PICKUP_HIGH(58,22, "Post Pickup High"),
    STOWED_AFTER_PICKUP_LOW(58, 2, "Post Pickup Low"),
    CLIMB(22,0, "Climb Stow"),
    STOWED(58,0, "Stowed"),
    STOWED_UP(58, 0, "Stowed up"),
    STOWED_UP_TELEOP(58, 0, "Stowed up Tele");
    
   public final double Arm;
   public final double Lift;
   public final String name;


    private SuperstructurePosition(double Arm,double Lift, String name ){
this.Arm = Arm;
this.Lift = Lift;
this.name = name;
    }

    public double getArm(){
        return Arm;

    }
    public double getLift(){
        return Lift;
    }
    public String getName(){
        return name;
    }
}
