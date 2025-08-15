Docker設定

歯車マークでUbuntuのスイッチを入れる。

![1755237975276](image/process/1755237975276.png)


```
Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running
```

対策

```
sudo update-alternatives --set iptables /usr/sbin/iptables-legacy
sudo update-alternatives --set ip6tables /usr/sbin/ip6tables-legacy
sudo service docker restart
```


![1755240456401](image/process/1755240456401.png)
