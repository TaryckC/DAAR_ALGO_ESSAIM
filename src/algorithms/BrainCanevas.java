/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/BrainCanevas.java 2014-10-19 buixuan.
 * ******************************************************/
package algorithms;

import characteristics.Parameters;
import robotsimulator.Brain;
import characteristics.IFrontSensorResult;
import robotsimulator.FrontSensorResult;

import java.util.ArrayList;

public class BrainCanevas extends Brain {
    private enum Task {
        TURN_LEFT,
        TURN_RIGHT,
        CLOSE_DISTANCE
    }

    private class TaskAttribute {
        double targetHeading;
        final int TARGET_STEPS = 100;
        private int currentStep;

        TaskAttribute(double targetHeading) {
            this.targetHeading = targetHeading;
        }

        TaskAttribute() {
            currentStep = 0;
        }

        // return pre-increment value
        public int incrementStep() {
            return currentStep++;
        }

        public int getCurrentStep() {
            return currentStep;
        }

        public boolean isTargetStepsReached() {
            return currentStep >= TARGET_STEPS;
        }
    }

    private final static double HEADING_PRECISION = 0.01;

        private ArrayList<Task> currentTasks;
    private TaskAttribute currentTaskAttribute;

  public void activate() {
      currentTasks = new ArrayList<>();
      currentTaskAttribute = null;
  }

  // return true if there is a task being executed and executes it
    // Useful to chain tasks
  public boolean callNextTask() {
    if (!currentTasks.isEmpty()) {
        switch (currentTasks.getFirst()) {
          case TURN_LEFT:
              turnLeft();
              return true;
          case TURN_RIGHT:
              turnRight();
              return true;
              case CLOSE_DISTANCE:
                  closeDistance();
                  return true;
        }
    }
    return false;
  }

  public void step() {
      // Execute current task if any
        if (callNextTask()) {
            return;
        }
        // No current task, decide next action
    if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
        // Try to determine which way to turn

        currentTasks.add(Task.CLOSE_DISTANCE);
        currentTasks.add(Task.TURN_RIGHT);
        callNextTask();
    } else {
        move();
    }
  }

  // LONGEMENT DES MURS
    private boolean turnLeft() {
      if (currentTaskAttribute == null) {
          // targetHeading = getHeading() - Math.PI / 2;
          this.currentTaskAttribute = new TaskAttribute(getHeading()  - 0.5 * Math.PI);
          sendLogMessage("Turning left.");
      }
        // Check if the turn is complete
        if (isHeadingReached(currentTaskAttribute.targetHeading)) {
            currentTasks.removeFirst();
            currentTaskAttribute = null;
            sendLogMessage("Turned right. Task complete.");
            callNextTask();
            return false;
        }
        else {
            stepTurn(Parameters.Direction.LEFT);
            return true;
        }
    }

    private boolean turnRight() {
        if (currentTaskAttribute == null) {
            // targetHeading = getHeading() - Math.PI / 2;
            this.currentTaskAttribute = new TaskAttribute(getHeading()  + 0.5 * Math.PI);
            sendLogMessage("Turning right.");
        }
        // Check if the turn is complete
        if (isHeadingReached(currentTaskAttribute.targetHeading)) {
            currentTasks.removeFirst();
            currentTaskAttribute = null;
            System.out.println("Turned right. Task complete.");
            sendLogMessage("Turned right. Task complete.");
            callNextTask();
            return false;
        }
        else {
            stepTurn(Parameters.Direction.RIGHT);
            return true;
        }
    }

    private boolean isHeadingReached(double target) {
      return Math.abs(Math.sin(getHeading() - target)) < HEADING_PRECISION;
    }

    private boolean closeDistance() {
        if (currentTaskAttribute == null) {
            this.currentTaskAttribute = new TaskAttribute();
            sendLogMessage("Closing distance.");
        }
        // Check if the move is complete
        if (currentTaskAttribute.isTargetStepsReached()) {
            currentTasks.removeFirst();
            currentTaskAttribute = null;
            sendLogMessage("Distance closed. Task complete.");
            callNextTask();
            return false;
        }
        else {
            move();
            currentTaskAttribute.incrementStep();
            return true;
        }
    }

}
