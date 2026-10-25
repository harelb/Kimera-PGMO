#include <gtest/gtest.h>

#include <set>

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

TEST(CommonFunctions, offsetFace) {
  const traits::Face orig{1, 2, 3};
  {  // test case 1
    const traits::Face expected{3, 4, 5};
    EXPECT_EQ(offsetFace(orig, 2), expected);
  }

  {  // test case 2
    const traits::Face expected{5, 6, 7};
    EXPECT_EQ(offsetFace(orig, 4), expected);
  }
}

TEST(CommonFunctions, remapFace) {
  std::map<size_t, size_t> remapping{{0, 1}, {1, 0}, {2, 4}, {3, 3}};
  const traits::Face orig{1, 2, 3};
  {  // test case 1
    const traits::Face expected{0, 4, 3};
    EXPECT_EQ(remapFace(orig, 0, remapping), expected);
  }

  {  // test case 2
    const traits::Face expected{2, 1, 5};
    EXPECT_EQ(remapFace(orig, 1, remapping), expected);
  }
}

TEST(CommonFunctions, robotPrefixes) {
  // every robot has a pose and a vertex prefix, the inverse maps agree, and no character is
  // used twice (a pose prefix of one robot must never be a vertex prefix of another)
  EXPECT_EQ(robot_id_to_prefix.size(), 15u);
  EXPECT_EQ(robot_id_to_vertex_prefix.size(), robot_id_to_prefix.size());
  std::set<char> used;
  for (const auto& [robot_id, prefix] : robot_id_to_prefix) {
    EXPECT_EQ(GetRobotPrefix(robot_id), prefix);
    EXPECT_EQ(robot_prefix_to_id.at(prefix), robot_id);
    const char vertex_prefix = GetVertexPrefix(robot_id);
    EXPECT_EQ(vertex_prefix_to_id.at(vertex_prefix), robot_id);
    EXPECT_TRUE(used.insert(prefix).second) << prefix;
    EXPECT_TRUE(used.insert(vertex_prefix).second) << vertex_prefix;
  }
  EXPECT_EQ(GetRobotPrefix(15), '\0');
}

TEST(CommonFunctions, allVerticesBelow) {
  traits::Face f{3, 4, 5};
  EXPECT_TRUE(allVerticesBelow(f, 6));
  EXPECT_FALSE(allVerticesBelow(f, 5));
  EXPECT_FALSE(allVerticesBelow(f, 3));
  EXPECT_FALSE(allVerticesBelow(f, 2));
}

TEST(CommonFunctions, checkFace) {
  const auto is_even = [](size_t x) { return x % 2 == 0; };
  const auto is_odd = [](size_t x) { return x % 2 == 1; };

  traits::Face f1{2, 4, 6};
  EXPECT_TRUE(checkFaceAll(f1, is_even));
  EXPECT_TRUE(checkFaceAny(f1, is_even));
  EXPECT_FALSE(checkFaceAll(f1, is_odd));
  EXPECT_FALSE(checkFaceAny(f1, is_odd));

  traits::Face f2{2, 3, 6};
  EXPECT_FALSE(checkFaceAll(f2, is_even));
  EXPECT_TRUE(checkFaceAny(f2, is_even));
  EXPECT_FALSE(checkFaceAll(f2, is_odd));
  EXPECT_TRUE(checkFaceAny(f2, is_odd));

  traits::Face f3{3, 2, 6};
  EXPECT_FALSE(checkFaceAll(f3, is_even));
  EXPECT_TRUE(checkFaceAny(f3, is_even));
  EXPECT_FALSE(checkFaceAll(f3, is_odd));
  EXPECT_TRUE(checkFaceAny(f3, is_odd));

  traits::Face f4{6, 2, 3};
  EXPECT_FALSE(checkFaceAll(f4, is_even));
  EXPECT_TRUE(checkFaceAny(f4, is_even));
  EXPECT_FALSE(checkFaceAll(f4, is_odd));
  EXPECT_TRUE(checkFaceAny(f4, is_odd));
}

TEST(CommonFunctions, applyToFace) {
  const traits::Face orig{1, 2, 3};
  const traits::Face expected{2, 4, 6};
  EXPECT_EQ(applyToFace(orig, [](auto x) { return 2 * x; }), expected);
}

}  // namespace kimera_pgmo
