# ros2_ws

ROS 2 practice workspace containing C++ and Python packages.

## Packages
- `my_cpp_pkg` — ROS 2 C++ node examples
- `my_py_pkg3` — ROS 2 Python node examples

## Workspace Structure
```text
ros2_ws/
├── src/
│   ├── my_cpp_pkg/
│   └── my_py_pkg3/
├── build/      # generated (not tracked)
├── install/    # generated (not tracked)
└── log/        # generated (not tracked)

EXTRAS:
The short version: your code was fine at the end — the main problem was an environment / package resolution issue, not your C++ logic.

What likely caused all of this
1) ros2 run was launching an older installed cpp_node

Your source file clearly had:

Timer callback triggered: ...

But ros2 run my_cpp_pkg cpp_node still printed:

Hello, world

That means ROS was not running the binary built from the file you were editing (or it was using an old installed copy still visible in your environment).

This usually happens because of:

old install/ contents

previously sourced workspace paths

another overlay/package path taking priority

stale AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH

You even saw warnings like:

path in AMENT_PREFIX_PATH doesn’t exist

That’s a strong clue that the shell environment had stale references.

2) Your ~/.bashrc had a broken ROS workspace line

You got this earlier:

bash: /home/sadab/ros2_ws/install/setup.bash: Permission denied

That usually means .bashrc had something like:

/home/sadab/ros2_ws/install/setup.bash

instead of:

source ~/ros2_ws/install/setup.bash

So every time you opened a terminal / ran source ~/.bashrc, the shell tried to execute setup.bash like a program (wrong) instead of sourcing it (correct).

That messed with your shell state and made debugging much more confusing.

3) Earlier build issue: undefined reference to main

That was a different issue and you actually fixed it.

That linker error means CMake was trying to build an executable, but the .cpp file being compiled didn’t contain a valid main() (or the wrong file was referenced). Once your source/CMake setup matched, the build succeeded.

So there were basically two separate problems:

build configuration mismatch (fixed)

runtime environment / old binary being executed (the confusing one)

Why it felt so confusing

Because all 3 layers can be “green” at once:

✅ file looks correct in VS Code

✅ colcon build succeeds

❌ ros2 run still prints old output

That happens when the runtime binary path != the file you just edited.

Final root cause (most likely)

If I had to summarize in one sentence:

Your shell environment was stale/broken (bad .bashrc + old ROS overlay paths), so ros2 run kept resolving to an older installed executable instead of the newly edited node.

How to avoid this next time (super practical)

Use this workflow whenever ROS behaves weirdly:

# new terminal
source /opt/ros/humble/setup.bash

cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

ros2 pkg prefix my_cpp_pkg
ros2 run my_cpp_pkg cpp_node

And if output still looks old, run the binary directly:

~/ros2_ws/install/my_cpp_pkg/lib/my_cpp_pkg/cpp_node

That instantly tells you whether the problem is:

build/install or

ROS package resolution

You actually did a very solid debugging job. The grep, find, and colcon list checks you ran were exactly the right moves.

EXTRA 2:

OOP Template for Your Nodes
Here’s a template you can use for all your nodes. Make sure you understand what each line does by rewatching (if needed) the previous lectures.

From now on I will start the nodes from this template so we don’t waste time always writing the same code at the beginning of each video.

OOP Python Code Template for Nodes
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class MyCustomNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()


OOP C++ Code Template for Nodes
#include "rclcpp/rclcpp.hpp"
 
class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

