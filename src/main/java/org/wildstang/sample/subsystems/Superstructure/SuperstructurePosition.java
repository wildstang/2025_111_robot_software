package org.wildstang.sample.subsystems.Superstructure;

public enum SuperstructurePosition {
//first value its arm second value lift position.
    CORAL_STATION_FRONT (9,0, "Coral Station Front"),
    CORAL_STATION_BACK(104,0, "Coral Station Back"),
    ALGAE_PRESTAGED(100,0, "Algae Prestaged"),
    ALGAE_REEF_LOW(87,4, "Algae Pickup Low"),
    ALGAE_REEF_HIGH(87,27, "Algae Pickup High"),
    CORAL_REEF_L1(36,22, "Reef L1"),
    CORAL_REEF_L2(36,27, "Reef L2"),
    CORAL_REEF_L3(36,48, "Reef L3"),
    CORAL_REEF_L4 (36,82, "Reef L4"),
    ALGAE_NET_FRONT(0,0, "Algae Net Front"),
    ALGAE_NET_BACK(0,0,"Algae Net Back"),
    ALGAE_PROCESSOR_BACK(22,0, "Algae Proc Back"),
    ALGAE_PROCESSOR_FRONT(100,0, "Algae Proc Front"),
    STOWED_AFTER_PICKUP_HIGH(58,27, "Post Pickup High"),
    STOWED_AFTER_PICKUP_LOW(58, 4, "Post Pickup Low"),
    CLIMB(22,0, "Climb Stow"),
    STOWED(87,0, "Stowed");
    
   public final int Arm;
   public final int Lift;
   public final String name;


    private SuperstructurePosition(int Arm,int Lift, String name ){
this.Arm = Arm;
this.Lift = Lift;
this.name = name;
    }

    public int getArm(){
        return Arm;

    }
    public int getLift(){
        return Lift;
    }
    public String getName(){
        return name;
    }
}
