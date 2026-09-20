// Copyright 2026 Sebastian Parra

#include "soft_glove_core/soft_glove_core.h"

#include "gtest/gtest.h"

TEST(ScaffoldLink, ProbeReturnsZero)
{
  EXPECT_EQ(soft_glove_core_scaffold_probe(), 0);
}
