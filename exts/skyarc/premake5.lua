-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

local ext = get_current_extension_info()

project_ext(ext)
repo_build.prebuild_link { "docs", ext.target_dir .. "/docs" }
repo_build.prebuild_link { "skyarc", ext.target_dir .. "/skyarc" }
