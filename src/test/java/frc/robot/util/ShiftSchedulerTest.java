package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class ShiftSchedulerTest {

  @Test
  public void testInitialState() {
    assertFalse(ShiftScheduler.isInitialized());
    assertEquals("PENDING", ShiftScheduler.getAutoResult());
    assertFalse(ShiftScheduler.hasWonAuto());
  }
}
