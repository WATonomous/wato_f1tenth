// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "my_controller/my_controller.hpp"

namespace my_controller
{

Command compute(const State & state, const nav_msgs::msg::Path & path, bool path_is_local, const Params & params)
{
  (void)state;
  (void)path;
  (void)path_is_local;
  (void)params;

  // TODO(you): your control law. Until then the car stays stopped.
  Command cmd;
  return cmd;
}

}  // namespace my_controller
