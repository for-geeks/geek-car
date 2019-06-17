# A Sample Framework Based On Cyber RT.

## Environment Build

### Source List

`sudo vim /etc/apt/source.list`

ARM
```
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse
```

X86
```
deb https://mirrors.aliyun.com/ubuntu/ bionic main restricted universe multiverse 
deb https://mirrors.aliyun.com/ubuntu/ bionic-security main restricted universe multiverse 
deb https://mirrors.aliyun.com/ubuntu/ bionic-updates main restricted universe multiverse 
deb https://mirrors.aliyun.com/ubuntu/ bionic-proposed main restricted universe multiverse 
deb https://mirrors.aliyun.com/ubuntu/ bionic-backports main restricted universe multiverse
```

Let's Install Basical Library and Tools:

Arm Plarform :

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
bash apollo.sh build_opt
```

## Have a try on Cyber RT


## Appendix I Cyber RT

- Bazel
- Protobuf
- Glog
- Gtest
- Fast-rtps
- Pcl
