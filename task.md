# 作業ログ
やったこと と 詰まったところを大まかに
## 0913
- opencv install  
libopencv-dev cmake をインストール

- 画像の表示を試してみる。  
[opencv参考ページ](https://www.qoosky.io/techs/ad1e4deb05)  
画像を置く場所はmain.cppのある場所じゃなく、buildフォルダがカレントディレクトリ。  
imgフォルダから指定するようにする。

- 実行はbuild ディレクトリがlsで見えるところで、
```
cmake ..
make 
./Cmakeのプロジェクト名
```

- vscodeでopencvのが見つからないよってエラー吐く。  
vscodeのC++設定ファイルにinclude pathを書くところがあるので、 そこにopencvのpathを追加する。
ubuntu22.04で`sudo apt install livopencv-dev`したときは、includepathが`/usr/include/opencv4`でした。

- eigenインストール  
EigenからDLしてきて /usr/local/include/eigen3にコピーする。

- firstプログラムを動かしてみる。
動かせた。コンパイルもヘッダー指定がもとからされているのか 何も指定しなくても動いた。

- Cmake使うときEigenの指定がfind_packageで必要かと思ったが、そももそも指定しなくてもコンパイルできてたのでいらなかった。  
そのままでコンパイルできなかった場合はよくわからない。

- まずは1点での処理を考える。