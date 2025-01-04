#include "stf/transform_tree.hpp"
#include <gtest/gtest.h>
#include <cmath>

class TransformTreeTest : public ::testing::Test {
protected:
    stf::TransformTree tf;

    // Helper function to create a translation transform
    static Eigen::Isometry3d makeTranslation(double x, double y, double z) {
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
        t.translate(Eigen::Vector3d(x, y, z));
        return t;
    }

    // Helper function to create a rotation transform around Z axis
    static Eigen::Isometry3d makeRotationZ(double angle_rad) {
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
        t.rotate(Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()));
        return t;
    }

    // Helper function to compare transforms with tolerance
    static void expectTransformsEqual(const Eigen::Isometry3d& expected,
                                      const Eigen::Isometry3d& actual, double tolerance = 1e-6) {
        // Compare translation
        EXPECT_NEAR(expected.translation().x(), actual.translation().x(), tolerance);
        EXPECT_NEAR(expected.translation().y(), actual.translation().y(), tolerance);
        EXPECT_NEAR(expected.translation().z(), actual.translation().z(), tolerance);

        // Compare rotation matrices
        Eigen::Matrix3d expected_rot = expected.rotation();
        Eigen::Matrix3d actual_rot = actual.rotation();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                EXPECT_NEAR(expected_rot(i, j), actual_rot(i, j), tolerance);
            }
        }
    }
};

TEST_F(TransformTreeTest, IdentityTransformForSameFrame) {
    auto result = tf.getTransform("frame_a", "frame_a");
    expectTransformsEqual(Eigen::Isometry3d::Identity(), result.transform);
    EXPECT_EQ("frame_a", result.path);
}

TEST_F(TransformTreeTest, SimpleTranslation) {
    auto t1 = makeTranslation(1.0, 2.0, 3.0);
    tf.setTransform("frame_a", "frame_b", t1);

    auto result = tf.getTransform("frame_a", "frame_b");
    expectTransformsEqual(t1, result.transform);
    EXPECT_EQ("frame_a/frame_b", result.path);
}

TEST_F(TransformTreeTest, ReverseTransform) {
    auto t1 = makeTranslation(1.0, 2.0, 3.0);
    tf.setTransform("frame_a", "frame_b", t1);

    EXPECT_NO_THROW({
        auto result = tf.getTransform("frame_b", "frame_a");
        expectTransformsEqual(t1.inverse(), result.transform);
        EXPECT_EQ("frame_b/frame_a", result.path);
    });
}

TEST_F(TransformTreeTest, ChainedTransforms) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeRotationZ(M_PI_2);  // 90 degrees

    tf.setTransform("frame_a", "frame_b", t1);
    tf.setTransform("frame_b", "frame_c", t2);

    auto result = tf.getTransform("frame_a", "frame_c");
    auto expected = t1 * t2;
    expectTransformsEqual(expected, result.transform);
    EXPECT_EQ("frame_a/frame_b/frame_c", result.path);

    result = tf.getTransform("frame_c", "frame_a");
    expectTransformsEqual(expected.inverse(), result.transform);
    EXPECT_EQ("frame_c/frame_b/frame_a", result.path);
}

TEST_F(TransformTreeTest, UpdateExistingTransform) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeTranslation(2.0, 0.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    tf.setTransform("frame_a", "frame_b", t2);  // Update existing transform

    auto result = tf.getTransform("frame_a", "frame_b");
    expectTransformsEqual(t2, result.transform);
    EXPECT_EQ("frame_a/frame_b", result.path);
}

TEST_F(TransformTreeTest, NonExistentTransform) {
    EXPECT_THROW(tf.getTransform("frame_x", "frame_y"), std::runtime_error);
}

TEST_F(TransformTreeTest, ComplexChain) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeRotationZ(M_PI_2);
    auto t3 = makeTranslation(0.0, 2.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    tf.setTransform("frame_b", "frame_c", t2);
    tf.setTransform("frame_c", "frame_d", t3);

    auto result = tf.getTransform("frame_a", "frame_d");
    auto expected = t1 * t2 * t3;
    expectTransformsEqual(expected, result.transform);
    EXPECT_EQ("frame_a/frame_b/frame_c/frame_d", result.path);

    result = tf.getTransform("frame_b", "frame_d");
    expected = t2 * t3;
    expectTransformsEqual(expected, result.transform);
    EXPECT_EQ("frame_b/frame_c/frame_d", result.path);

    result = tf.getTransform("frame_a", "frame_c");
    expected = t1 * t2;
    expectTransformsEqual(expected, result.transform);
    EXPECT_EQ("frame_a/frame_b/frame_c", result.path);
}

TEST_F(TransformTreeTest, PreventDirectCycle) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    EXPECT_THROW(tf.setTransform("frame_b", "frame_a", t1), std::runtime_error);
}

TEST_F(TransformTreeTest, PreventIndirectCycle) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeTranslation(0.0, 1.0, 0.0);
    auto t3 = makeTranslation(0.0, 0.0, 1.0);

    tf.setTransform("frame_a", "frame_b", t1);
    tf.setTransform("frame_b", "frame_c", t2);
    EXPECT_THROW(tf.setTransform("frame_c", "frame_a", t3), std::runtime_error);
}

TEST_F(TransformTreeTest, PreventSelfTransform) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    EXPECT_THROW(tf.setTransform("frame_a", "frame_a", t1), std::runtime_error);
}

TEST_F(TransformTreeTest, AllowUpdateExistingEdge) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeTranslation(2.0, 0.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    EXPECT_NO_THROW(tf.setTransform("frame_a", "frame_b", t2));

    auto result = tf.getTransform("frame_a", "frame_b");
    expectTransformsEqual(t2, result.transform);
    EXPECT_EQ("frame_a/frame_b", result.path);
}

TEST_F(TransformTreeTest, CycleDetectionWithExistingPath) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeTranslation(0.0, 1.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    tf.setTransform("frame_b", "frame_c", t2);

    EXPECT_THROW(tf.setTransform("frame_c", "frame_b", t1), std::runtime_error);
    EXPECT_THROW(tf.setTransform("frame_b", "frame_a", t2), std::runtime_error);
    EXPECT_THROW(tf.setTransform("frame_c", "frame_a", t1), std::runtime_error);
}

TEST_F(TransformTreeTest, CycleDetectionInBranch) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeTranslation(0.0, 1.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    tf.setTransform("frame_a", "frame_c", t2);

    EXPECT_THROW(tf.setTransform("frame_c", "frame_b", t1), std::runtime_error);
    EXPECT_THROW(tf.setTransform("frame_b", "frame_c", t2), std::runtime_error);
}

TEST_F(TransformTreeTest, LongChainCycleDetection) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    tf.setTransform("frame_b", "frame_c", t1);
    tf.setTransform("frame_c", "frame_d", t1);

    EXPECT_THROW(tf.setTransform("frame_d", "frame_a", t1), std::runtime_error);
    EXPECT_THROW(tf.setTransform("frame_d", "frame_b", t1), std::runtime_error);
    EXPECT_THROW(tf.setTransform("frame_c", "frame_a", t1), std::runtime_error);
}

TEST_F(TransformTreeTest, ValidPathUpdates) {
    auto t1 = makeTranslation(1.0, 0.0, 0.0);
    auto t2 = makeTranslation(2.0, 0.0, 0.0);

    tf.setTransform("frame_a", "frame_b", t1);
    EXPECT_NO_THROW(tf.setTransform("frame_a", "frame_b", t2));

    auto result = tf.getTransform("frame_a", "frame_b");
    expectTransformsEqual(t2, result.transform);
    EXPECT_EQ("frame_a/frame_b", result.path);
}
