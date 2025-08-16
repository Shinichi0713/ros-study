はい、できます。

`catkin_make` や `catkin build`（catkin_tools）の場合でやり方が少し違います。

---

## 1. **catkin_make の場合**

`catkin_make` は基本的にワークスペース内のすべてのパッケージをビルドしますが、以下の方法で特定パッケージだけを対象にできます。

### 特定パッケージだけをビルド

```bash
catkin_make --pkg <package_name>
```

### 複数パッケージをビルド

```bash
catkin_make --pkg <package1> <package2>
```

※ただし、このとき依存関係が未ビルドの場合は自動で解決してくれません。

そのため、依存パッケージも明示的にビルドする必要があります。

---

## 2. **catkin_tools (catkin build) を使う場合**

`catkin build` の方が柔軟で、パッケージ単位のビルドが簡単です。

### 特定パッケージだけをビルド

```bash
catkin build <package_name>
```

### 複数パッケージ

```bash
catkin build <package1> <package2>
```

### 依存関係ごとビルド

```bash
catkin build --deps <package_name>
```

---

## 3. **フォルダごとにビルドしたい場合**

ROS のパッケージは基本的に `CMakeLists.txt` と `package.xml` を持つディレクトリ単位で管理されるので、

「フォルダごとにビルド」というのは **パッケージごとにビルド** に相当します。

もし `src/` の中に複数のパッケージがあり、そのうち一部のフォルダだけビルドしたいなら：

* `catkin_make --pkg` か `catkin build` で対象を指定する
* あるいは、一時的に不要なフォルダを `src/` から別の場所へ移動する（古典的なやり方）
