package com.kuriosityrobotics.centerstage.math;

import org.junit.jupiter.api.Test;

class MathUtilTest {
   @Test
   void clampTest() {
	  assert (MathUtil.clamp(-1, 0, 100) == 0);
	  assert (MathUtil.clamp(50, 0, 100) == 50);
	  assert (MathUtil.clamp(1001, 0, 100) == 100);
   }
}
