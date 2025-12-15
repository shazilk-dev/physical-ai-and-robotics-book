#!/usr/bin/env python3
"""
Test Suite 1: URDF Validity and Syntax
Lab 2: Humanoid URDF

Tests:
1. URDF file exists
2. URDF parses correctly (valid XML)
3. All required links exist
4. All required joints exist
5. No duplicate link/joint names
6. Tree structure is valid (no loops)
"""

import unittest
import xml.etree.ElementTree as ET
import os
import sys


class TestURDFValidity(unittest.TestCase):
    """Test URDF file validity and structure."""

    @classmethod
    def setUpClass(cls):
        """Load URDF file once for all tests."""
        # Assume URDF is in ../starter/urdf/humanoid.urdf relative to this file
        test_dir = os.path.dirname(os.path.abspath(__file__))
        cls.urdf_path = os.path.join(test_dir, '..', 'starter', 'urdf', 'humanoid.urdf')

        if not os.path.exists(cls.urdf_path):
            raise FileNotFoundError(f"URDF file not found: {cls.urdf_path}")

        # Parse URDF
        try:
            cls.tree = ET.parse(cls.urdf_path)
            cls.root = cls.tree.getroot()
        except ET.ParseError as e:
            raise ValueError(f"Failed to parse URDF: {e}")

    def test_01_urdf_file_exists(self):
        """Test that URDF file exists."""
        self.assertTrue(
            os.path.exists(self.urdf_path),
            f"❌ URDF file not found at: {self.urdf_path}"
        )

    def test_02_valid_xml(self):
        """Test that URDF is valid XML."""
        self.assertIsNotNone(
            self.root,
            "❌ Failed to parse URDF as XML"
        )
        self.assertEqual(
            self.root.tag,
            'robot',
            "❌ Root element must be <robot>"
        )

    def test_03_robot_has_name(self):
        """Test that robot has a name attribute."""
        robot_name = self.root.get('name')
        self.assertIsNotNone(
            robot_name,
            "❌ Robot must have a 'name' attribute"
        )
        self.assertEqual(
            robot_name,
            'simple_humanoid',
            f"❌ Expected robot name 'simple_humanoid', got '{robot_name}'"
        )

    def test_04_all_required_links_exist(self):
        """Test that all required links are defined."""
        required_links = [
            'torso',
            'head',
            'left_shoulder_virtual_link',
            'left_upper_arm',
            'left_forearm',
            'right_shoulder_virtual_link',
            'right_upper_arm',
            'right_forearm'
        ]

        # Get all link names
        links = self.root.findall('link')
        link_names = [link.get('name') for link in links]

        for required in required_links:
            self.assertIn(
                required,
                link_names,
                f"❌ Required link '{required}' not found in URDF. "
                f"Found links: {link_names}"
            )

    def test_05_all_required_joints_exist(self):
        """Test that all required joints are defined."""
        required_joints = [
            'neck',
            'left_shoulder_pitch',
            'left_shoulder_roll',
            'left_elbow',
            'right_shoulder_pitch',
            'right_shoulder_roll',
            'right_elbow'
        ]

        # Get all joint names
        joints = self.root.findall('joint')
        joint_names = [joint.get('name') for joint in joints]

        for required in required_joints:
            self.assertIn(
                required,
                joint_names,
                f"❌ Required joint '{required}' not found in URDF. "
                f"Found joints: {joint_names}"
            )

    def test_06_no_duplicate_link_names(self):
        """Test that all link names are unique."""
        links = self.root.findall('link')
        link_names = [link.get('name') for link in links]

        duplicates = set([name for name in link_names if link_names.count(name) > 1])
        self.assertEqual(
            len(duplicates),
            0,
            f"❌ Duplicate link names found: {duplicates}"
        )

    def test_07_no_duplicate_joint_names(self):
        """Test that all joint names are unique."""
        joints = self.root.findall('joint')
        joint_names = [joint.get('name') for joint in joints]

        duplicates = set([name for name in joint_names if joint_names.count(name) > 1])
        self.assertEqual(
            len(duplicates),
            0,
            f"❌ Duplicate joint names found: {duplicates}"
        )

    def test_08_torso_has_geometry(self):
        """Test that torso link has visual geometry."""
        torso = self.root.find(".//link[@name='torso']")
        self.assertIsNotNone(torso, "❌ Torso link not found")

        visual = torso.find('visual')
        self.assertIsNotNone(
            visual,
            "❌ Torso must have <visual> geometry (TODO 1)"
        )

        geometry = visual.find('geometry')
        self.assertIsNotNone(
            geometry,
            "❌ Torso visual must have <geometry>"
        )

        box = geometry.find('box')
        self.assertIsNotNone(
            box,
            "❌ Torso should use <box> geometry"
        )

    def test_09_joints_have_limits(self):
        """Test that revolute joints have limit tags."""
        joints = self.root.findall(".//joint[@type='revolute']")

        for joint in joints:
            joint_name = joint.get('name')
            limit = joint.find('limit')

            self.assertIsNotNone(
                limit,
                f"❌ Joint '{joint_name}' must have <limit> tag"
            )

            # Check limit attributes
            self.assertIsNotNone(
                limit.get('lower'),
                f"❌ Joint '{joint_name}' limit must have 'lower' attribute"
            )
            self.assertIsNotNone(
                limit.get('upper'),
                f"❌ Joint '{joint_name}' limit must have 'upper' attribute"
            )

    def test_10_tree_structure_validity(self):
        """Test that kinematic tree has no loops (all links have max 1 parent)."""
        joints = self.root.findall('joint')

        # Map child link → parent link
        child_to_parent = {}

        for joint in joints:
            parent_elem = joint.find('parent')
            child_elem = joint.find('child')

            if parent_elem is not None and child_elem is not None:
                parent_link = parent_elem.get('link')
                child_link = child_elem.get('link')

                if child_link in child_to_parent:
                    self.fail(
                        f"❌ Link '{child_link}' has multiple parents. "
                        f"This creates a loop in the kinematic tree!"
                    )

                child_to_parent[child_link] = parent_link

        print(f"✅ Valid kinematic tree with {len(child_to_parent)} child links")


def run_tests():
    """Run all tests and print summary."""
    suite = unittest.TestLoader().loadTestsFromTestCase(TestURDFValidity)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print("\n" + "="*70)
    if result.wasSuccessful():
        print("✅ ALL URDF VALIDITY TESTS PASSED!")
        print(f"   - {result.testsRun} tests run")
        print(f"   - 0 failures")
        print(f"   - 0 errors")
        print("\n   Your URDF structure is valid!")
    else:
        print("❌ SOME TESTS FAILED")
        print(f"   - {result.testsRun} tests run")
        print(f"   - {len(result.failures)} failures")
        print(f"   - {len(result.errors)} errors")
        print("\n   Fix the errors above and re-run the tests.")
    print("="*70 + "\n")

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
