package org.wildstang.framework.auto.steps;

import org.wildstang.framework.auto.AutoStep;

public class LambdaStep extends AutoStep {

    // Function Interface
    public interface Procedure {
        void invoke();
    }

    private Procedure function;
    private String name;

    public LambdaStep(Procedure function, String name) {
        this.function = function;
        this.name = name;
    }

    @Override
    public void initialize() {
        function.invoke();
        this.setFinished();
    }

    @Override
    public void update() {
    }

    @Override
    public String toString() {
        return name;
    }
    
}
