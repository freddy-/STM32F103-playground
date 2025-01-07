#!/bin/bash

for (( c=1; c<=50000; c++ ))
do
  echo "Tentativa $c"

  curl 'http://192.168.0.39:8088/api/led/on' \
-H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.7' \
-H 'Accept-Language: pt-BR,pt;q=0.9,en-US;q=0.8,en;q=0.7,es;q=0.6,ru;q=0.5,tr;q=0.4,fr;q=0.3,de;q=0.2,ja;q=0.1' \
-H 'Cache-Control: max-age=0' \
-H 'Connection: keep-alive' \
-H 'Upgrade-Insecure-Requests: 1' \
-H 'User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/131.0.0.0 Safari/537.36' \
--insecure;

  curl 'http://192.168.0.39:8088/api/led/off' \
-H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.7' \
-H 'Accept-Language: pt-BR,pt;q=0.9,en-US;q=0.8,en;q=0.7,es;q=0.6,ru;q=0.5,tr;q=0.4,fr;q=0.3,de;q=0.2,ja;q=0.1' \
-H 'Cache-Control: max-age=0' \
-H 'Connection: keep-alive' \
-H 'Upgrade-Insecure-Requests: 1' \
-H 'User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/131.0.0.0 Safari/537.36' \
--insecure;


done