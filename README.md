ddynamic_reconfigure2
==================================================

## 概要
本パッケージは，ROS2ノードに対して動的に変更可能なノードパラメータを簡便に設定するためのインタフェースを提供します．これは，ROS1における[ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure)に相当するものですが，C++（ノードを[rclcpp](https://github.com/ros2/rclcpp)で記述）とPython（ノードを[rclpy](https://github.com/ros2/rclpy)で記述）の両方に対応しています．

## 背景
ROS1では，ROSネットワーク内で使われる全パラメータが一箇所(roscoreのパラメータサーバ)で管理されており，その値をノードの実行中に対話的に変更するために[dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure)という特殊な仕組みが使われてきました．`dynamic_reconfigure`を使ってノードを開発すれば，動的に変更したいパラメータを定義し，その値が取り得るレンジを指定できます．しかし，それはノードのビルド時に行われる必要があり，ノードの起動後にパラメータの定義やレンジの設定をすることはできませんでした．

これは，大きな制約となることがあります．例えば，USBカメラには画像サイズ，画素フォーマット，ホワイトバランス，露光時間等の撮影パラメータを動作中に変更できるものがありますが，どのようなパラメータが変更可能なのか，どの範囲の値を指定できるかはカメラの機種によって異なり，それらを知るにはカメラを実際に起動してハードウェアに問い合わせなければなりません．そのため，撮影パラメータを実行中に変更できる汎用的なUSBカメラのドライバノードを開発しようとすると，それらをノードパラメータとして管理し，取り得る値をノードの初期化時に調べてレンジとして指定できるような機構が欲しくなります．`dynamic_reconfigure`ではこのような状況に対応できません．

これを解決するために開発されたのが[ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure)で，動的に変更可能なパラメータとそれが取り得るレンジをノード内で定義・管理するサーバ機能が提供されるようになりました．ただし，サポートされる開発言語はC++のみで，[rospy](https://wiki.ros.org/rospy)で書かれたPythonノードには対応していませんでした．

その後．ROS1からROS2への移行に際し，パラメータの扱いが大きく変わりました．
- ROS1では全パラメータがroscoreのパラメータサーバで集中管理されていたのに対し，ROS2では各ノードがサーバ機能を持ち，自分が使うパラメータだけを管理するようになった．
- 各ノードは，自分が管理するパラメータを外部から参照・変更することを許す機能を標準的に備えるようになった．

このため，`dynamic_reconfigure`や`ddynamic_reconfigure`に相当する機能は[rclcpp](https://github.com/ros2/rclcpp)や[rclpy](https://github.com/ros2/rclpy)のノードが直接提供するようになり，上記のような状況にも個々のノードの開発者側で対処できるようになりました．しかし，`rclcpp`や`rclpy`で提供されるのは，パラメータの宣言やその値を変更するためのコールバックを設定するような比較的低レベルのAPIであり，必ずしも使い勝手の良いものではありません．

本パッケージは，ROS1の`ddynacmic_reconfigure`に類似した簡便なインタフェースでノードパラメータとそのレンジを定義するAPIを提供します．これによって，動的に変更可能なパラメータを有するノードの開発が容易になります．

## インストール
### 注意
本パッケージは，ROS2ノードの[ParameterEventHandler](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1ParameterEventHandler.html)を用いて実装されています．これは[Jazzy](https://docs.ros.org/en/jazzy/index.html)以降では[C++](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)と[Python](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-Python.html)の両方でサポートされていますが，[Humble](https://docs.ros.org/en/humble/index.html)以前では[C++](https://docs.ros.org/en/humble/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)でしかサポートされません．したがって，本パッケージをPythonで利用するには`Jazzy`以降のdistributionが必要です．


### インストール手順
`github`から`ddynamic_reconfigure2`を入手し，`develop`ブランチを取り出します．
```bash
$ cd catkin_ws/src
$ git clone git@github.com:Automation-Research-Team/ddynamic_reconfigure2.git
$ cd ddynamic_reconfigure2
$ git checkout develop
```
そして，ワークスペース全体をコンパイルしてください．
```bash
$ cd catkin_ws
$ colcon build
```

### rqt_reconfigure
本パッケージは，ノードの実行中にそのパラメータを対話的に変更するためのものですので，その操作用GUIである[rqt_reconfigure](https://index.ros.org/p/rqt_reconfigure/)をインストールしておいてください．ROS2標準リポジトリから
```bash
$ sudo apt install ros-jazzy-rqt-reconfigure
```
でインストールできますが，パラメータの階層的グループ分けや有限個の候補から値を選択する列挙型のパラメータに対応していないので，それらに対応するように[修正したバージョン](https://github.com/Automation-Research-Team/rqt_reconfigure)をインストールすることをお薦めします．

## テスト
### C++版
```bash
$ ros2 launch ddynamic_reconfigure2 test.launch.py
```
によって[テストプログラム](./src/testnode.cpp)と`rqt_reconfigure`が起動します．テストプログラム中では以下のパラメータが定義されており，1秒ごとにその値がコンソールに表示されます．
- **param_b**: bool型パラメータ
- **numeric.param_i64**: 整数型パラメータ，`numeric`グループに所属
- **numeric.param_d**: 浮動小数点数型パラメータ, `numeric`グループに所属
- **string.param_s**: 文字列型パラメータ，`string`グループに所属
- **params_b**: bool型パラメータの配列
- **numeric.params_i64**: 整数型パラメータの配列，`numeric`グループに所属
- **numeric.params_d**: 浮動小数点数型パラメータの配列, `numeric`グループに所属
- **string.params_s**: 文字列型パラメータの配列，`string`グループに所属
- **numeric.enum_param_d**: 有限個の候補から値を選ぶ浮動小数点数型パラメータ, `numeric`グループに所属
- **string.enum_param_s**: 有限個の候補から値を選ぶ文字列型パラメータ，`string`グループに所属

`rqt_reconfigure`からこれらの値を変更すると，表示もそれに合わせて変化することがわかるでしょう．

### Python版（Jazzy以降）
以下によってC++版と同様の[テストプログラム](./scripts/pytestnode.py)を起動できます．
```bash
$ ros2 launch ddynamic_reconfigure2 pytest.launch.py
```

## APIの使い方
ROS2のパラメータには，ROS1に比べて以下のような違いがあります．
- パラメータは，必ずノード内で宣言してから使わなければならない．
- パラメータの型は，[9種類に限定されている](https://docs.ros2.org/latest/api/rcl_interfaces/msg/ParameterType.html)．
- パラメータ名を階層化する場合の区切り文字は，`/`ではなく`.`である．
- パラメータの初期値は，宣言時もしくはノード起動時に与えたYAML形式のconfigurationファイルによって指定される．両方を指定した場合は，後者が優先される．
- パラメータには，その型ばかりでなく，取り得る値の範囲（レンジ）や候補値（列挙型），外部からの変更の可否などの[属性](https://docs.ros2.org/latest/api/rcl_interfaces/msg/ParameterDescriptor.html)が付与され，値の変更時にその条件が満たされているかチェックされる．

パラメータの型がbool，整数，浮動小数点，文字列およびそれらの配列に限定されているので，ROS1のように，異なる型が混在したリストや文字列をキーとした辞書を1つのパラメータとして扱うことはできません．例えば，`numeric.param_i64`と`numeric.param_d`というそれぞれ整数型と浮動小数点型の2つのパラメータを定義した場合，これらは`numeric`というグループに所属しますが，`numeric`そのものを`param_i64`と`param_d`という2つのキーを持つ辞書型のパラメータとして扱うことはできません．

パラメータを使うノードを開発する場合は，これらの点に留意する必要があります．

### C++
使い方の例は，[テストプログラム](./src/testnode.cpp)を参照してください．

開発するノードに[class DDynamicReconfigure2\<NODE\>](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp)型のメンバ変数を持たせることにより，パラメータやそのレンジを設定できるようになります．
```c++
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>                      

class TestNode : public rclcpp::Node
{
  public:
    TestNode(const rclcpp::NodeOptions& options=rclcpp::NodeOptions())  ;
    ... 

  private:
    DDynamicReconfigure<> _ddr;
    int64_t				  _param_i64;
};

TestNode::TestNode(const rclcpp::NodeOptions& options)                          
    :rclcpp::Node("testnode", options),                                         
     _ddr(rclcpp::Node::SharedPtr(this)),                                       
     _param_i64(4),
     ...          
{
    ...
}   
```

ある変数をノードパラメータに結びつけ，その値を外部から変更可能にするためには，[template \<class T\> DDynamicReconfigure<NODE>::registerVariable(const std::string& name, T* variable, const std::string& description, const param_range<T>& range)](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp#L243-#L253)を使います．
- **name**: パラメータ名
- **variable**: パラメータの値を保持する変数へのポインタ．ノード起動時のパラメータconfigurationファイルで指定されていなければ，呼び出し時におけるこの変数の値がパラメータの初期値となる．
- **description**: パラメータの説明を与える任意のテキスト
- **range**: パラメータのレンジ

例えば，`int64_t`型の変数`_param_i64`を`numeric.param_i64`という名前のパラメータとして定義し，そのレンジを-4以上10以下で刻み幅2とするには，
```c++
{
    _ddr.registerVariable("numeric.param_i64", &_param_i64,
			              "parameter of int64_t type", {-4, 10, 2});
}
```
とします．これで，変数`_param_i64`の値を`rqt_reconfigure`を用いて外部から操作できます．刻み幅を省略するとその値は0となり，これは最大値と最小値の間で任意の値をとれることを意味します．テンプレートパラメータ`T`は，

パラメータを，C++の変数ではなく，C++の関数に結びつけてノードの状態を変更したいこともあるでしょう．その場合は，[template \<class T\> DDynamicReconfigure\<NODE\>::registerVariable(const std::string& name, const T& current_value, const std::function<void(const T&)>& cb, const std::string& description, const param_range\<T\>& range)](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp#L255-#L270)を使います．
- **name**: パラメータ名
- **current_value**: ノード起動時のパラメータconfigurationファイルで指定されていない場合に設定されるパラメータの初期値
- **cb**: パラメータ変更時に呼ばれるコールバック関数
- **description**: パラメータの説明を与える任意のテキスト
- **range**: パラメータのレンジ
