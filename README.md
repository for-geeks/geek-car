# Geek Car, An Autonomous Application Based On Cyber RT

![Action Status](https://github.com/for-geeks/geek-car/workflows/Geek%20Style/badge.svg)

<img src="images/red.jpg" style="zoom:100%;display: inline-block; float:middle"/>

As the basic layer of the Apollo autonomous platform, Cyber provides data reliability and timeliness for the application (perception, planning, control, etc.), based on which you can create your own application layer according to your own situation.

<img src="images/system.png" style="zoom:80%;display: inline-block; float:middle"/>

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

## Environment Setup

### Code Clone

1. Clone main codes:  `git clone git@github.com:for-geeks/geek-car.git`
2. Update submodule:
```bash
git submodule init
git submodule update
```
### Installation
Let's Install Basical Library and Tools:

ARM Platform :

```bash
sudo bash docker/build/cyber.aarch64.sh
```
X86 Platform:

```bash
sudo bash docker/build/cyber.x86_64.sh
```

## Build Geek car

```bash
bash apollo.sh build
```

## Have a try

More details about teminal.

More details about develop toolschain you can find here：[CyberRT_Developer_Tools](https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md)


## Copyright and License
Geekcar is provided under the MIT License.
