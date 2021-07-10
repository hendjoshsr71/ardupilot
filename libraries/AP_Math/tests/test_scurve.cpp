#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/SCurve.h>

// Tests that T4 is positive for the following solutions

// FIX ME nly  one of these is tested here
// solution = 2 - t6 t4 t2 = 0 1 0
// solution = 7 - t6 t4 t2 = 1 1 1
TEST(LinesScurve, test_calculate_path)
{
    float Jm_out, t2_out, t4_out, t6_out;
    SCurve::calculate_path(0.300000012, 19.4233513, 0, 5.82700586, 188.354691, 2.09772229,
                           Jm_out, t2_out, t4_out, t6_out);
    EXPECT_FLOAT_EQ(Jm_out, 19.423351);
    EXPECT_FLOAT_EQ(t2_out, 0.0);
    EXPECT_FLOAT_EQ(t4_out, 0.0);
    EXPECT_FLOAT_EQ(t6_out, 0.0);
}

// Test that no velocity change returns
TEST(LinesScurve, test_calculate_path)
{
    float Jm_out = 1.1, t2_out = 2.2, t4_out = 3.3, t6_out = 4.4;
    SCurve::calculate_path(0.0, 0.0, 10.0, 0.0, 1.0, 2.09772229,
                           Jm_out, t2_out, t4_out, t6_out);
    // How to check if it returns immeadiately before setting the floats without bool output
    EXPECT_FLOAT_EQ(Jm_out, 1.1);
    EXPECT_FLOAT_EQ(t2_out, 2.2);
    EXPECT_FLOAT_EQ(t4_out, 3.2);
    EXPECT_FLOAT_EQ(t6_out, 4.4);
}

// Test solution = 0 - t6 t4 t2 = 0 0 0
// Test (fabsf(Am) < Jm * tj): TRUE && (Vm <= V0 + 2.0f * Am * tj) = TRUE




// Test solution = 0 - t6 t4 t2 = 0 0 0
// Test (fabsf(Am) < Jm * tj): TRUE && (L <= 4.0f * V0 * tj + 4.0f * Am * sq(tj)) = TRUE



// Test solution = 2 - t6 t4 t2 = 0 1 0
// Test (fabsf(Am) < Jm * tj) && (Vm <= V0 + 2.0f * Am * tj) = FALSE && (L <= 4.0f * V0 * tj + 4.0f * Am * sq(tj)) = FALSE


// solution = 5 - t6 t4 t2 = 1 0 1
// Test (fabsf(Am) < Jm * tj): FALSE  && (0 < a1) : TRUE



// solution = 5 - t6 t4 t2 = 1 0 1
// Test (fabsf(Am) < Jm * tj): FALSE  && (L < 1.0f / (Jm_sq) * (Am_sq * Am + Am * Jm * (V0 * 2.0f + Am * tj * 2.0f)) + V0 * tj * 2.0f + Am * (tj_sq)) : TRUE


// solution = 7 - t6 t4 t2 = 1 1 1
// Test (fabsf(Am) < Jm * tj): FALSE  && (0 < a1) : FALSE
// && (L < 1.0f / (Jm_sq) * (Am_sq * Am + Am * Jm * (V0 * 2.0f + Am * tj * 2.0f)) + V0 * tj * 2.0f + Am * (tj_sq)) : TRUE






AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
