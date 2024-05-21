# coordinate_transform

## 環境設定
- opencv install  
libopencv-dev cmake をインストール

- vscodeでopencvのが見つからないよってエラー吐くときは、  
vscodeのC++設定ファイルにinclude pathを書くところがあるので、 そこにopencvのpathを追加する。
ubuntu22.04で`sudo apt install libopencv-dev`したときは、includepathが`/usr/include/opencv4`でした。

- eigenインストール  
[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)のget itからDLするなりwgetするなりしてきて 展開した中のEigenディレクトリを/usr/local/include/eigen3にコピーする。

- Cmake使うときEigenの指定がfind_packageで必要かと思ったが、そももそも指定しなくてもコンパイルできてたのでいらなかった。  
そのままでコンパイルできなかった場合はよくわからない。


## 実行
build ディレクトリがlsで見えるところで、
```
cmake ..
make 
./Cmakeのプロジェクト名
```


- 画像の表示を試してみる。  
[opencv参考ページ](https://www.qoosky.io/techs/ad1e4deb05)  
画像を置く場所はmain.cppのある場所じゃなくimgフォルダから指定するぞ。

## point_getter
java がインストールされてなかったら
```
sudo apt update
java -version

インストールされてなかったら
sudo apt install default-jre
```

プログラム自体は、pointgetterのディレクトリで
```
java -jar pointgetter.jar
```
で

```
5
249 114
247 200
122 175
315 174
28 30



※ 先頭行    ：座標の数
```
で出力される。


