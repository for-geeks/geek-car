# A Sample Framework Based On Cyber RT

As the basic layer of the Apollo autonomous platform, Cyber provides data reliability and timeliness for the application (sensing, planning, control, etc.), based on which you can create your own application layer according to your own situation.

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

```bash
# in apollo_lite directory
bash apollo.sh build_opt
```

## Have a try on Cyber RT


## Appendix I：Cyber RT

- Bazel
- Protobuf
- Glog
- Gtest
- Fast-rtps
- Pcl
