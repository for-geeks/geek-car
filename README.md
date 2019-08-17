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

**Note:If you use ARM plarform, Please use debug not optimization mode**
```bash
# in apollo_lite directory
bash apollo.sh build
```

## Realsense Configure
Auto get default realsense device, auto update serial number in file `modules/sensors/conf/sensors.conf` with field serial_number.The flags field is `FLAGS_serial_number`, you can get it if you want use.

how to get this serial_number? use command `rs-enumerate-devices` provided by Intel.You will get something like :

```bash
geek-car@geekcar-desktop:~/geek_lite$ rs-enumerate-devices 
Device info: 
    Name                          : 	Intel RealSense T265
    Serial Number                 : 	909212110229
    Firmware Version              : 	0.0.18.5715
    Physical Port                 : 	vid_8087 pid_0B37 bus_2 port_3
    Product Id                    : 	0B37
    Product Line                  : 	T200

Stream Profiles supported by Tracking Module
 Supported modes:
    stream       resolution      fps       format
    Fisheye 1	  848x800	@ 30Hz	   Y8
    Fisheye 2	  848x800	@ 30Hz	   Y8
    Gyro	 N/A		@ 200Hz	   MOTION_XYZ32F
    Accel	 N/A		@ 62Hz	   MOTION_XYZ32F
    Pose	 N/A		@ 200Hz	   6DOF

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

More details about develop toolschain you can find here：[CyberRT_Developer_Tools](https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md)


## Appendix I: Update

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

1. `ImportError:No module named proto.unit_test_pb2`:

`source scripts/apollo_base.sh`

2. setup soft link to source directory

```bash
sudo ln -s path/to/geek_lite /apollo
```
