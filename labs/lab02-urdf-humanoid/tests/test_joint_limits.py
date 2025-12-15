#!/usr/bin/env python3
"""
Test Suite 2: Joint Limits Validation
Lab 2: Humanoid URDF

Verifies that all joint limits match specifications.
"""

import unittest
import xml.etree.ElementTree as ET
import os
import math


class TestJointLimits(unittest.TestCase):
    """Test that joint limits match specifications."""

    @classmethod
    def setUpClass(cls):
        """Load URDF file."""
        test_dir = os.path.dirname(os.path.abspath(__file__))
        cls.urdf_path = os.path.join(test_dir, '..', 'starter', 'urdf', 'humanoid.urdf')

        cls.tree = ET.parse(cls.urdf_path)
        cls.root = cls.tree.getroot()

    def get_joint_limits(self, joint_name):
        """Get lower and upper limits for a joint."""
        joint = self.root.find(f".//joint[@name='{joint_name}']")
        if joint is None:
            return None, None

        limit = joint.find('limit')
        if limit is None:
            return None, None

        lower = float(limit.get('lower', '0'))
        upper = float(limit.get('upper', '0'))
        return lower, upper

    def test_neck_limits(self):
        """Test neck joint limits (±90°)."""
        lower, upper = self.get_joint_limits('neck')

        self.assertAlmostEqual(
            lower,
            -math.pi/2,
            delta=0.01,
            msg=f"❌ Neck lower limit should be -π/2 (-90°), got {lower:.3f}"
        )
        self.assertAlmostEqual(
            upper,
            math.pi/2,
            delta=0.01,
            msg=f"❌ Neck upper limit should be +π/2 (+90°), got {upper:.3f}"
        )

    def test_left_shoulder_pitch_limits(self):
        """Test left shoulder pitch limits (-45° to +180°)."""
        lower, upper = self.get_joint_limits('left_shoulder_pitch')

        self.assertAlmostEqual(
            lower,
            -math.pi/4,
            delta=0.01,
            msg=f"❌ Left shoulder pitch lower limit should be -π/4 (-45°), got {lower:.3f}"
        )
        self.assertAlmostEqual(
            upper,
            math.pi,
            delta=0.01,
            msg=f"❌ Left shoulder pitch upper limit should be π (+180°), got {upper:.3f}"
        )

    def test_left_shoulder_roll_limits(self):
        """Test left shoulder roll limits (0° to +90°)."""
        lower, upper = self.get_joint_limits('left_shoulder_roll')

        self.assertAlmostEqual(
            lower,
            0.0,
            delta=0.01,
            msg=f"❌ Left shoulder roll lower limit should be 0°, got {lower:.3f}"
        )
        self.assertAlmostEqual(
            upper,
            math.pi/2,
            delta=0.01,
            msg=f"❌ Left shoulder roll upper limit should be π/2 (+90°), got {upper:.3f}"
        )

    def test_left_elbow_limits(self):
        """Test left elbow limits (0° to +135°)."""
        lower, upper = self.get_joint_limits('left_elbow')

        self.assertAlmostEqual(
            lower,
            0.0,
            delta=0.01,
            msg=f"❌ Left elbow lower limit should be 0°, got {lower:.3f}"
        )
        self.assertAlmostEqual(
            upper,
            2.356,  # 135° in radians
            delta=0.01,
            msg=f"❌ Left elbow upper limit should be 2.356 (+135°), got {upper:.3f}"
        )

    def test_right_shoulder_roll_limits(self):
        """Test right shoulder roll limits (-90° to 0°, mirrored)."""
        lower, upper = self.get_joint_limits('right_shoulder_roll')

        self.assertAlmostEqual(
            lower,
            -math.pi/2,
            delta=0.01,
            msg=f"❌ Right shoulder roll lower limit should be -π/2 (-90°), got {lower:.3f}"
        )
        self.assertAlmostEqual(
            upper,
            0.0,
            delta=0.01,
            msg=f"❌ Right shoulder roll upper limit should be 0°, got {upper:.3f}"
        )


if __name__ == '__main__':
    unittest.main(verbosity=2)
