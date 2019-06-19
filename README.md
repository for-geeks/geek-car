# A Sample Framework Based On Cyber RT

As the basic layer of the Apollo autonomous platform, Cyber provides data reliability and timeliness for the application (perception, planning, control, etc.), based on which you can create your own application layer according to your own situation.

Key benefits of using Apollo Cyber RT:

- Accelerate development
  + Well defined task interface with data fusion
  + Array of development tools
  + Large set of sensor drivers
- Simplify deployment
  + Efficient and adaptive message communication
  + Configurable user level scheduler with resource awareness
  + Portable with fewer dependencies
- Empower your own autonomous vehicles
  + The default open source runtime framework
  + Building blocks specifically designed for autonomous driving
  + Plug and play your own AD system

## Environment Build

Let's Install Basical Library and Tools:

ARM Platform :

```bash
cd docker/build 
sudo bash cyber.aarch64.sh
```
X86 Platform:

```bash
cd docker/build
sudo bash cyber.x86_64.sh
```

## Build Framework

**Note:Please use debug not optimization mode**
```bash
# in apollo_lite directory
bash apollo.sh build
```

## Have a try on Cyber RT

### Writing a Simple Talker and Listener (C++)

1.Create Talker

```cpp
#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Chatter;

int main(int argc, char *argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create talker node
  auto talker_node = apollo::cyber::CreateNode("talker");
  // create talker
  auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
  Rate rate(1.0);
  while (apollo::cyber::OK()) {
    static uint64_t seq = 0;
    auto msg = std::make_shared<Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq++);
    msg->set_content("Hello, apollo!");
    talker->Write(msg);
    AINFO << "talker sent a message!";
    rate.Sleep();
  }
  return 0;
}
```

2.Create Listener
```cpp
#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"

void MessageCallback(
    const std::shared_ptr<apollo::cyber::examples::proto::Chatter>& msg) {
  AINFO << "Received message seq-> " << msg->seq();
  AINFO << "msgcontent->" << msg->content();
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");
  // create listener
  auto listener =
      listener_node->CreateReader<apollo::cyber::examples::proto::Chatter>(
          "channel/chatter", MessageCallback);
  apollo::cyber::WaitForShutdown();
  return 0;
}
```

### Writing a Simple Talker and Listener (Python)

Talker:
```python
from cyber_py import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark
"""Module for example of talker."""

import time
import sys

sys.path.append("../")


def test_talker_class():
    """
    Test talker.
    """
    msg = ChatterBenchmark()
    msg.content = "py:talker:send Alex!"
    msg.stamp = 9999
    msg.seq = 0
    print msg
    test_node = cyber.Node("node_name1")
    g_count = 1

    writer = test_node.create_writer("channel/chatter",
                                     ChatterBenchmark, 6)
    while not cyber.is_shutdown():
        time.sleep(1)
        g_count = g_count + 1
        msg.seq = g_count
        msg.content = "I am python talker."
        print "=" * 80
        print "write msg -> %s" % msg
        writer.write(msg)


if __name__ == '__main__':
    cyber.init("talker_sample")
    test_talker_class()
    cyber.shutdown()
```

Listener:
```python
# -*- coding: utf-8 -*-
"""Module for example of listener."""

import sys

sys.path.append("../")
from cyber_py import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark


def callback(data):
    """
    Reader message callback.
    """
    print "=" * 80
    print "py:reader callback msg->:"
    print data
    print "=" * 80


def test_listener_class():
    """
    Reader message.
    """
    print "=" * 120
    test_node = cyber.Node("listener")
    test_node.create_reader("channel/chatter",
                            ChatterBenchmark, callback)
    test_node.spin()

if __name__ == '__main__':
    cyber.init()
    test_listener_class()
    cyber.shutdown()
```

### Building a module based on Component

Cyber RT framework is built based on the concept of component. As a basic building block of Apollo Cyber RT framework, each component contains a specific algorithm module which process a set of data inputs and generate a set of outputs.

In order to successfully create and launch a new component, there are four essential steps that need to happen:

- Set up the component file structure
- Implement the component class
- Set up the configuration files
- Launch the component

## Set up the component file structure
Please create the following files, assumed under the directory of `/apollo/cyber/examples/common_component_example/`:

- Header file: common_component_example.h
- Source file: common_component_example.cc
- Build file: BUILD
- DAG dependency file: common.dag
- Launch file: common.launch

## Implement the component class

### Implement component header file
To implement `common_component_example.h`:

- Inherit the Component class
- Define your own `Init` and `Proc` functions. Proc function needs to specify its input data types
- Register your component classes to be global by using
`CYBER_REGISTER_COMPONENT`

```cpp
#include <memory>
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Driver;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class CommonComponentSample : public Component<Driver, Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
};

CYBER_REGISTER_COMPONENT(CommonComponentSample)
```

### Implement the source file for the example component

For `common_component_example.cc`, both `Init` and `Proc` functions need to be implemented.

```cpp
#include "cyber/examples/common_component_example/common_component_example.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

bool CommonComponentSample::Init() {
  AINFO << "Commontest component init";
  return true;
}

bool CommonComponentSample::Proc(const std::shared_ptr<Driver>& msg0,
                               const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
```

### Create the build file for the example component

Create bazel BUILD file.

```bash
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_component_example.so",
    deps = [":common_component_example_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "common_component_example_lib",
    srcs = [
        "common_component_example.cc",
    ],
    hdrs = [
        "common_component_example.h",
    ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
```
## Set up the configuration files

### Configure the DAG dependency file

To configure the DAG dependency file (common.dag), specify the following items as below:

 - Channel names: for data input and output
 - Library path: library built from component class
 - Class name: the class name of the component

```bash
# Define all coms in DAG streaming.
    component_config {
    component_library : "/apollo/bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so"
    components {
        class_name : "CommonComponentSample"
        config {
            name : "common"
            readers {
                channel: "/apollo/prediction"
            }
            readers {
                channel: "/apollo/test"
            }
        }
      }
    }
```

### Configure the launch file

To configure the launch (common.launch) file, specify the following items:

  - The name of the component
  - The dag file you just created in the previous step.
  - The name of the process which the component runs within

```bash
<cyber>
    <component>
        <name>common</name>
        <dag_conf>/apollo/cyber/examples/common_component_example/common.dag</dag_conf>
        <process_name>common</process_name>
    </component>
</cyber>
```

## Launch the component

Build the component by running the command below:

```bash
bash apollo.sh build_opt
```

Note: make sure the example component builds fine

Then configure the environment:

```bash
source cyber/setup.bash
```

There are two ways to launch the component:

- Launch with the launch file (recommended)

```bash
cyber_launch start cyber/examples/common_component_example/common.launch
```

- Launch with the DAG file

```bash
mainboard -d cyber/examples/common_component_example/common.dag
```

It's really simple, More details you can find here：[CyberRT_API_for_Developers](https://github.com/ApolloAuto/apollo/tree/master/docs/cyber/CyberRT_API_for_Developers.md)


### CyberRT Developer Tools

When you use Cyber, you must live with these tools, like ros sub-command famlily. 

#### Cyber_visualizer

`cyber_visualizer` is a visualization tool for displaying the channel data in Apollo Cyber RT.

#### Cyber_monitor
The command line tool `cyber_monitor` provides a clear view of the list of real time channel information Apollo Cyber RT in the terminal.

#### Cyber_recorder

#### Cyber_node

#### Cyber_service

#### rosbag_to_record
rosbag_to_record is a tool which can convert rosbag to recorder file provided by Apollo Cyber RT. Now the tool support following channel:

More details about develop toolschain you can find here：[CyberRT_Developer_Tools](https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md)

## Appendix I：Cyber RT Dependence  

- Bazel
- Protobuf
- Glog
- Gtest
- Fast-rtps
- Pcl

## Appendix II: Update

```bash
# open coredump
ulimit -c unlimited

# core dump to target directory
if [ -e /proc/sys/kernel ]; then
    echo "/apollo/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern > /dev/null
fi

# install protobuf
pip install protobuf

```

## FAQ

1.`ImportError:No module named proto.unit_test_pb2`:

`source scripts/apollo_base.sh`
