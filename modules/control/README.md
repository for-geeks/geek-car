# Control Module


1、给Arduino设备(`/dev/ttyACM0`)读取权限，只需要将用户比如apollo添加到用户组dialout：
```bash
sudo usermod -aG　dialout apollo
```


可能遇到undefied symbol 的解决办法:

```bash
nm -u *.so
// 确定未定义的symbol
```

```bash
c++filt symbol
```

查找对应未定义的内容；


参考：
https://blog.csdn.net/stpeace/article/details/76561814
https://medium.com/fcamels-notes/%E8%A7%A3%E6%B1%BA-linux-%E4%B8%8A-c-c-%E7%9A%84-undefined-symbol-%E6%88%96-undefined-reference-a80ee8f85425