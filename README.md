![GitHub release (latest by date)](https://img.shields.io/github/v/release/Automation-Research-Team/ddynamic_reconfigure2)
![GitHub](https://img.shields.io/github/license/Automation-Research-Team/ddynamic_reconfigure2)

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
### 動作環境
本パッケージは，`Ubuntu-24.04` + [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)の下で動作確認しています．
| ROS 2 Distribution | Humble                                                                                                                                                                      | Jazzy                                                                                                                                                                    |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Build Status       | [![humble-build](https://github.com/OpenHRC/OpenHRC/actions/workflows/humble-build.yaml/badge.svg)](https://github.com/OpenHRC/OpenHRC/actions/workflows/humble-build.yaml) | [![jazzy-build](https://github.com/Automation-Research-Team/ddynamic_reconfigure2/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/Automation-Research-Team/ddynamic_reconfigure2actions/workflows/jazzy-build.yaml) |

#### 注意
本パッケージは，ROS2ノードの[ParameterEventHandler](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1ParameterEventHandler.html)を用いて実装されています．これは[Jazzy](https://docs.ros.org/en/jazzy/index.html)以降では[C++](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)と[Python](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-Python.html)の両方でサポートされていますが，[Humble](https://docs.ros.org/en/humble/index.html)以前では[C++](https://docs.ros.org/en/humble/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)でしかサポートされません．したがって，本パッケージをPythonで利用するには`Jazzy`以降のdistributionが必要です．


### インストール手順
まず最初に，[nlohmann-json](https://github.com/nlohmann/json)をインストールします．
```
sudo apt install nlohmann-json3-dev
```
次に，`github`から`ddynamic_reconfigure2`を入手し，`develop`ブランチを取り出します．
```bash
cd ros2_ws/src
git clone git@github.com:Automation-Research-Team/ddynamic_reconfigure2.git
cd ddynamic_reconfigure2
git checkout develop
```
そして，ワークスペース全体をコンパイルしてください．
```bash
cd ros2_ws
colcon build
```

### rqt_reconfigure
本パッケージは，ノードの実行中にそのパラメータを対話的に変更するためのものですので，その操作用GUIである[rqt_reconfigure](https://index.ros.org/p/rqt_reconfigure/)をインストールしておいてください．ROS2標準リポジトリから
```bash
sudo apt install ros-jazzy-rqt-reconfigure
```
でインストールできますが，パラメータの階層的グループ分けや有限個の候補から値を選択する列挙型のパラメータに対応していないので，それらに対応するように[修正したバージョン](https://github.com/Automation-Research-Team/rqt_reconfigure)をインストールすることをお薦めします．

## テスト
### C++版
```bash
ros2 launch ddynamic_reconfigure2 test.launch.py
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

`rqt_reconfigure`からこれらの値を変更すると，表示もそれに合わせて変化することがわかるでしょう．また，これらの値の初期値は，[パラメータ設定用YAMLファイル](./config/test.yaml)で指定されています．

### Python版（Jazzy以降）
以下によってC++版と同様の[テストプログラム](./scripts/pytestnode.py)を起動できます．
```bash
ros2 launch ddynamic_reconfigure2 pytest.launch.py
```

## C++ APIの使い方 
ROS2のパラメータには，ROS1に比べて以下のような違いがあります．
- パラメータは，必ずノード内で宣言してから使わなければならない．
- パラメータの型は，[9種類に限定されている](https://docs.ros2.org/latest/api/rcl_interfaces/msg/ParameterType.html)．`ddynamic_reconfigure2`は，このうちバイト列(`PARAMETER_BYTE_ARRAY`)を除く8種類をサポートする．
- パラメータ名を階層化する場合の区切り文字は，`/`ではなく`.`である．
- パラメータの初期値は，宣言時に指定するか，もしくはノード起動時に与えたYAML形式のconfigurationファイルによって指定する．両方を指定すると，後者が優先される．
- パラメータには，その型ばかりでなく，取り得る値の範囲（レンジ）や候補値（列挙型），外部からの変更の可否などの[属性](https://docs.ros2.org/latest/api/rcl_interfaces/msg/ParameterDescriptor.html)が付与され，値の変更時にその条件が満たされているかチェックされる．

パラメータの型が`bool`，整数，浮動小数点，文字列およびそれらの配列に限定されているので，ROS1のように，異なる型が混在したリストや文字列をキーとした辞書を1つのパラメータとして扱うことはできません．例えば，`numeric.param_i64`と`numeric.param_d`というそれぞれ整数型と浮動小数点型の2つのパラメータを定義した場合，これらは`numeric`というグループに所属しますが，`numeric`そのものを`param_i64`と`param_d`という2つのキーを持つ辞書型のパラメータとして扱うことはできません．パラメータを使うノードを開発する際には，これらの点に留意する必要があります．

以下では，[テストプログラム](./src/testnode.cpp)を例題としてC++ APIの使い方を説明します．

### パラメータ管理機能のセットアップ
開発するノードに[class DDynamicReconfigure\<NODE\>](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp#L192-L193)型のメンバ変数を持たせることにより，パラメータやそのレンジを設定する準備が整います．テンプレートパラメータ`NODE`には，ノードの種類に応じて[rclcpp::Node](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html)（省略可）または[rclcpp_lifecycle::LifecycleNode](https://docs.ros2.org/latest/api/rclcpp_lifecycle/classrclcpp__lifecycle_1_1LifecycleNode.html)を指定します．
```c++
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>

namespace ddynamic_reconfigure2
{
class TestNode : public rclcpp::Node
{
  public:
    TestNode(const rclcpp::NodeOptions& options=rclcpp::NodeOptions())  ;
    ...

  private:
    DDynamicReconfigure<>   _ddr;
    int64_t				    _param_i64;
    double                  _param_d;
    std::string				_enum_param_s;
    ...
};

TestNode::TestNode(const rclcpp::NodeOptions& options)
    :rclcpp::Node("testnode", options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _param_i64(4),
     _param_d(0.5),
     _enum_param_s("Two")
     ...
{
    ...
}
}
```


### パラメータをC++変数に直接結びつける
ノードパラメータをあるC++変数に結びつけてその値を外部から変更可能にするには，[次の関数](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp#L206-#L209)を呼びます．
```c++
template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerVariable(const std::string& name,
                                            T* variable,
                                            const std::string& description="",
                                            const param_range<T>& range={});
```
- **name**: パラメータ名．階層化する場合の区切り文字は`.`
- **variable**: パラメータの値を保持する変数へのポインタ．テンプレートパラメータ`T`が取り得る型は[std::is_arithmetic\<T\>.value ](https://cpprefjp.github.io/reference/type_traits/is_arithmetic.html)が`true`となる型，`std::string`型もしくはそれらを要素とする`std::vector`型である．ノード起動時のパラメータconfigurationファイルでこのパラメータの値が指定されていなければ，呼び出し時におけるこの変数の値がパラメータの初期値となる
- **description**: パラメータの説明を与える任意のテキスト
- **range**: パラメータのレンジを最小値，最大値，刻み幅の3つ組で指定する．刻み幅を省略するとその値は0となり，これは最大値と最小値の間で任意の値をとれることを意味する．テンプレートパラメータ`T`が`bool`, `std::string`, `std::vector`の場合は無効

例えば，`int64_t`型の変数`_param_i64`を`numeric.param_i64`という名前のパラメータとして定義し，そのレンジを-4以上10以下で刻み幅2とするには，
```c++
    _ddr.registerVariable("numeric.param_i64", &_param_i64,
			              "parameter of int64_t type", {-4, 10, 2});
```
とします．これで，変数`_param_i64`の値を`rqt_reconfigure`を用いて外部から操作できます．

### パラメータをC++のコールバック関数に結びつける
パラメータを，C++の変数ではなく，C++の関数に結びつけてノードの状態を変更したいこともあるでしょう．その場合は，[次の関数](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp#L210-#L215)を使います．
```c++
template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerVariable(const std::string& name,
                                            const T& current_value,
                                            const std::function<void(const T&)>& cb,
                                            const std::string& description="",
                                            const param_range<T>& range={});
```

- **name**: パラメータ名．階層化する場合の区切り文字は`.`
- **current_value**: パラメータの初期値．Tが取り得る型は[std::is_arithmetic\<T\>.value ](https://cpprefjp.github.io/reference/type_traits/is_arithmetic.html)が`true`となる型，`std::string`型もしくはそれらを要素とする`std::vector`型である．ノード起動時のパラメータconfigurationファイルでこのパラメータの値が指定されていない場合に有効
- **cb**: パラメータ変更時に呼ばれるコールバック関数．外部から与えられたパラメータ更新値が引数として渡される．lambda関数も可
- **description**: パラメータの説明を与える任意のテキスト
- **range**: パラメータのレンジを最小値，最大値，刻み幅の3つ組で指定する．刻み幅を省略するとその値は0となり，これは最大値と最小値の間で任意の値をとれることを意味する．テンプレートパラメータ`T`が`bool`, `std::string`, `std::vector`の場合は無効

例えば，
```c++
    _ddr.registerVariable<double>("numeric.param_d", _param_d,
			                      [this](const double& x){ this->_param_d = x; },
			                      "parameter of double type", {-1.0, 2.0});
```
とすると，パラメータ`numeric.param_d`の値を外部から変更すると，それがここで指定したlambda関数に渡されて変数`_param_d`に代入されます．つまり前項の`numeric.param_i64`の例と同じ動作ですが，もっと複雑な処理をコールバックに行わせることも可能です．

### パラメータが取り得る値を有限個の候補値に限定する
[次の関数](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp#L216-#L221)を使うと，パラメータをあるC++変数に結びつけるとともに，その値を`enum_dict`に与えた有限個の候補値に限定することができます．
```c++
template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerEnumVariable(const std::string& name,
                                                T* variable,
                                                const std::string& description="",
                                                const std::map<std::string, T>& enum_dict={},
                                                const std::string& enum_description="");
```
- **name**: パラメータ名．階層化する場合の区切り文字は`.`
- **variable**: パラメータの値を保持する変数へのポインタ．Tが取り得る型は[std::is_arithmetic\<T\>.value ](https://cpprefjp.github.io/reference/type_traits/is_arithmetic.html)が`true`となる型，`std::string`型もしくはそれらを要素とする`std::vector`型である．ノード起動時のパラメータconfigurationファイルでこのパラメータの値が指定されていない場合に有効
- **description**: パラメータの説明を与える任意のテキスト
- **enum_dict**: 候補値の名前をkeyとし値をvalueとする辞書
- **enum_description**: 各候補値の説明を与える任意のテキスト

例えば．
```c++
    _ddr.registerEnumVariable("string.enum_param_s", &_enum_param_s,
	            		      "enum parameter of string type",
	            		      {{"one", "One"}, {"two", "Two"}, {"three", "Three"}},
	            		      "one/two/three");
```
とすれば，パラメータ`string.enum_param_s`を変数`_enum_param_s`に結びつけ，その値を`One`, `Two`, `Three`のいずれかに限定できます．

また．パラメータを変数ではなくコールバック関数に結びつけた上で，その取り得る値を有限個に限定したい場合は，[次の関数](./include/ddynamic_reconfigure2/ddynamic_reconfigure2.hpp#L222-#L229)を使います．
```c++
template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerEnumVariable(const std::string& name,
                                                const T& current_value,
                                                const std::function<void(const T&)>& cb,
                                                const std::string& description="",
                                                const std::map<std::string, T>& enum_dict={},
                                                const std::string& enum_description="");
```
## Python APIの使い方（Jazzy以降）
以下では，[テストプログラム](./scripts/pytestnode.py)を例題としてPython APIの使い方を説明します．

### パラメータ管理機能のセットアップ
開発するノードに[class DDynamicReconfigure](./ddynamic_reconfigure2/server.py#L46)型のメンバ変数を持たせることにより，パラメータやそのレンジを設定する準備が整います．
```python
from rclpy.node                   import Node
from ddynamic_reconfigure2.server import DDynamicReconfigure

class TestNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self._param_i64    = 4
        self._enum_param_d = 2.1
        ...
        self._ddr = DDynamicReconfigure(self)
```
### パラメータをPythonのコールバック関数に結びつける
`DDynamicReconfigure`の次の[メンバ関数](./ddynamic_reconfigure2/server.py#L54-#L55)を使うと，パラメータをPythonのコールバック関数に結びつけることができます．
```python
def register_variable(self, param_name, current_value, cb,
                      description='', min_value=None, max_value=None, step=0)
```
- **name**: パラメータ名．階層化する場合の区切り文字は`.`
- **current_value**: パラメータの初期値．取り得る型は`bool`, `int`, `float`, `str`もしくはそれらを要素とするシーケンス型である．ノード起動時のパラメータconfigurationファイルでこのパラメータの値が指定されていない場合に有効
- **cb**: パラメータ変更時に呼ばれるコールバック関数．外部から与えられたパラメータ更新値が引数として渡される．lambda関数も可
- **description**: パラメータの説明を与える任意のテキスト
- **min_value**: パラメータの最小値．`current_value`の型が`int`, `float`の場合のみ有効
- **max_value**: パラメータの最大値．`current_value`の型が`int`, `float`の場合のみ有効
- **step**: パラメータの刻み幅．省略するとその値は0となり，これは最大値と最小値の間で任意の値をとれることを意味する．`current_value`の型が`int`, `float`の場合のみ有効

例えば，
```python
self._ddr.register_variable('numeric.param_i64', self._param_i64,
                            lambda x: setattr(self, '_param_i64', x),
                            'parameter of int64_t type', -4, 10, 2)
```
とすれば，パラメータ`numeric.param_i64`が定義され，その値を外部から変更するとそれがlambda関数に渡されてクラス`TestNode`のメンバ変数`self._param_i64`に代入されます．

上記のようにlambda関数を用いれば簡単にPython変数を操作できるので，C++版のようなパラメータを直接変数に結びつけるAPIはありません．

### パラメータが取り得る値を有限個の候補値に限定する
`DDynamicReconfigure`の次の[メンバ関数](./ddynamic_reconfigure2/server.py#L61-#L62)を使うと，パラメータをPythonのコールバック関数に結びつけるとともに，その値を`enum_dict`に与えた有限個の候補値に限定することができます．
```python
def register_enum_variable(self, param_name, current_value, cb,
                           description, enum_dict, enum_description='')
```
- **name**: パラメータ名．階層化する場合の区切り文字は`.`
- **current_value**: パラメータの初期値．取り得る型は`bool`, `int`, `float`, `str`もしくはそれらを要素とするシーケンス型である．ノード起動時のパラメータconfigurationファイルでこのパラメータの値が指定されていない場合に有効
- **cb**: パラメータ変更時に呼ばれるコールバック関数．外部から与えられたパラメータ更新値が引数として渡される．lambda関数も可
- **description**: パラメータの説明を与える任意のテキスト
- **enum_dict**: 候補値の名前をkeyとし値をvalueとする辞書
- **enum_description**: 各候補値の説明を与える任意のテキスト

例えば，
```python
self._ddr.register_enum_variable('numeric.enum_param_d', self._enum_param_d,
                                 lambda x: setattr(self, '_enum_param_d', x),
                                 'enum parameter of double type',
                                 {'low': 1.0, 'middle': 2.1, 'high': 3.2},
                                 'low/middle/high')
```
とすれば，パラメータ`numeric.enum_param_d`が定義され，その値を外部から変更するとそれがlambda関数に渡されてクラス`TestNode`のメンバ変数`self._enum_param_d`に代入されます．このとき，取り得る値は`1.0`, `2.1`, `3.2`のいずれかに限定されます．

上記のようにlambda関数を用いれば簡単にPython変数を操作できるので，C++版のようなパラメータを直接変数に結びつけるAPIはありません．
