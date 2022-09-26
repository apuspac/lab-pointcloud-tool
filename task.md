# 作業ログ
やったこと と 詰まったところを大まかに
## 環境設定
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

## 回転行列Rを計算する
- 3点での処理でコアになる計算部分をつくる。  
logはissueの#3から見れる。

- ベクトルの内積と外積と普通の積の違いに注意。  
ドット積かなと思って相関行列Cを作ってたけど ドット積ならスカラーになるから行列にはならないよね。という話。  
気を付けよう。

- 直交行列になっているかを確認する
logはissue #5

## 有効桁数は常に確認
今やっていることは数値計算。誤差がでないとおかしいことをやっているので、型の有効桁数をちゃんと意識してやろう。

C++ double 

