FROM geekstyle/geek_lite:geek_lite-x86_64-18.04-20200425_1848

LABEL version="1.0"

ENV DEBIAN_FRONTEND=noninteractive

RUN adduser --disabled-password --gecos '' geek

WORKDIR /geek-car
