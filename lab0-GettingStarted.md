# 图形学物理仿真 Tutorial for Lab 0 

## Part 1: 课程实践（Lab 和 Project） 简介

欢迎大家选修图形学物理仿真课程。我们这学期共需要大家完成紧密围绕课程内容的 3 个 Lab，分别对应这门课程的三大主题：

 1. 刚体
 2. 流体
 3. 弹性体

我们的课程 Lab **不是代码填空**，大家需要自行设计代码结构，从新建文件开始实现模拟部分，因此希望同学们尽早动手。

除此之外，大家也会在学期末完成一项课程 Project。我们会提供一些 Project 题目供大家挑选，同时也很期待看到大家在课堂上找到自己在物理模拟领域感兴趣的方向，并以此作为你的课程 Project。

本学期的 Lab 基于 [vcx](https://gitee.com/pku-vcl/vcx2024/tree/master/) 代码库，大家可以去[可视计算与交互](https://vcl.pku.edu.cn/course/vci)课程了解更多关于代码库的应用。

我们的 Lab 支持 Windows, MacOS 和 Linux 的大多数发行版。如果你需要关于 Lab 的任何帮助，欢迎联系课程助教。

## Lab 0: Build the Codebase

### 1 - Merge Handout Packs

课程lab 0 提供的压缩包解压将建立一个名为 `vcx-sim-master` 的目录：这就是你这个学期做所有 Lab 的目录。
首先你需要按照 [Git 官网](https://git-scm.com/) 的指引安装 Git。


### 2 - Prepare Compiler

我们的 Lab 需要一个支持 C++20 标准的编译器。推荐使用以下编译器：

- Visual Studio 2019 以上版本，推荐 Visual Studio 2022
- GCC 10 以上版本，推荐 GCC 12
- XCode 13 以上版本
- Clang 13 以上版本

建议使用最新版本的相应编译器。

### 3 - Prepare Git and xmake

我们的 Lab 使用 xmake ([Home](https://xmake.io/)) 作为构建工具，而 xmake 依赖 Git 完成包管理等核心功能，请确保已经安装了 [Git](https://git-scm.com/) 。通过 [xmake安装说明](https://xmake.io/#/guide/installation) 中对于你的平台的描述，安装 xmake。完成安装后，可以在终端中运行 `xmake --version` 来确认安装正确。

接下来，你只需要在终端中进入 `vcx-sim-master` 目录（下同），然后在命令行中输入 `xmake` 并执行，你就会看到 xmake 自动识别你的平台，下载所有依赖库并完成编译和链接；注意 Windows 下使用 gitbash 终端或者 Anaconda Powershell Prompt 可能无法正常编译，请使用 Windows PowerShell；注意这里可能遇到一些网络问题，可以参见FAQ中网络错误的部分。

继续执行 `xmake run lab0`，如果一切顺利，你会看到一个界面，通过界面可以切换显示方框中的五个 Case。

<img src="./assets/images/lab0-case1.png" alt="case-1" style="zoom: 50%;" />
<img src="./assets/images/lab0-case2.png" alt="case-2" style="zoom: 50%;" />
<img src="./assets/images/lab0-case3.png" alt="case-3" style="zoom: 50%;" />
<img src="./assets/images/lab0-case4.png" alt="case-4" style="zoom: 50%;" />
<img src="./assets/images/lab0-case5.png" alt="case-5" style="zoom: 50%;" />

Case 1 和 Case 2 展示了二维作图，以及图形学常见的画三角形。

Case 3 展示了如何渲染一个三维盒子，你可以基于此完成刚体的 Lab。同时 Case 3 还实现了简单的交互，除了使用鼠标左右键以及WASDEQ移动相机以及滚轮缩放，Case 3 还实现了按住 Alt 键用鼠标左键拖动物体。你可以基于此实现自己的交互。

Case 4 展示了隐式弹簧质点模型，你可以借鉴这个例子中的仿真流程。

Case 5 展示了流体的渲染，这个例子中流体粒子做简单的简谐运动。现有基础的流体场景搭建，你需要在lab2中更改`Simulator`进行对应算法的实践。

在编译过程中，你可能遇到一些问题。我们为常见的问题提供了 xmake FAQ（见后文）。如果它无法解决你的问题，欢迎联系课程助教。

### 4 - Prepare for IDEs

接下来，你可能会想要让你喜欢的 IDE 了解 xmake 项目，以提供智能提示，调试器集成等功能：

 -  Visual Studio: 
    执行 `xmake project -a x64 -k vsxmake ./build`。
    你会在 `vcx/build/vsxmake20xx` 目录下找到 .sln 解决方案文件。

 -  XCode:
    执行 `xmake project -k xcode ./build`。
    你会在 `vcx/build/xcode` 目录下找到 XCode 项目文件。

 -  VS Code（图形化配置方案）：
    
    1. 首先执行 `xmake project -k compile_commands ./.vscode` 。
    2. 安装 C/C++ 插件与 XMake 插件。
    3. 选中顶部的 `View -> Command Palette...` ，输入 `XMake:` ，选择 `XMake: Update Intellisense` 。
    4. 选中顶部的 `View -> Command Palette...` ，输入 `C/C++:` ，选择 `C/C++: Edit Configurations (UI)` 。
    5. 选中 `C++ standard` 项，修改设置为 `C++20` 。
    6. 拉到最下方，点开 `Advanced Settings` ，在 `Compile Commands` 一栏输入 `${workspaceFolder}/.vscode/compile_commands.json`
    7. 返回 cpp 文件，现在 VS Code 应该已经能提供智能提示等功能了。
    
 -  VS Code（命令行配置方案）：
    首先执行 `xmake project -k compile_commands ./.vscode` 。
    然后你需要在 `vcx/.vscode` 目录下新建一个名为 `c_cpp_properties.json` 的文件，并写入：
    
    ```json
    { "configurations": [ { "name": "Default", "compileCommands": "${workspaceFolder}/.vscode/compile_commands.json" } ], "version": 4 }
    ```
    
    这样 VS Code 就能找到刚才命令生成的 `compile_commands.json` 并用它来帮助理解 C++ 项目了。

### xmake FAQ

- Q. 在 Windows 的 PowerShell 中输入 `xmake` 后显示「xmake : 无法将“xmake”项识别为 cmdlet、函数、脚本文件或可运行程序的名称」怎么办？

- A. 确保你已经通过 powershell 安装了 xmake；如果是通过下载 exe 安装包安装的 xmake，需要按照官网说明手动配置环境变量。

- Q. 在命令行中输入 `xmake` 后显示「note: xmake.lua not found, try generating it」怎么办？

- A. 确保命令行当前所在的文件夹是 `vcx-sim-master` ，如果不是，通过 cd 指令转到正确的路径。

- Q. 首次在命令行中输入 `xmake` 后需要安装一些包，此时报错下载失败并且能够看到「we can also download these packages manually」及「error: curl: (56) Recv failure: Connection was reset」字样怎么办？

- A. 这是因为访问 Github 时遇到了网络问题，有两种解决方案：
  - 打开本地代理，使用命令行设置好环境变量 `HTTPS_PROXY="127.0.0.1:<port>"`，之后在命令行中运行xmake
  
    + 例如，当你使用wallesspku时，默认端口一般是7890，此时在命令行中运行：
  
    + MacOS/Linux
  
      ```shell
      HTTPS_PROXY="127.0.0.1:7890" xmake
      ```
  
    + Windows cmd
  
      ```shell
      set HTTPS_PROXY=127.0.0.1:7890
      xmake
      ```
  
    + Windows Powershell
  
      ```shell
      $env:HTTPS_PROXY = "127.0.0.1:7890"
      xmake
      ```
  
  - 可运行 `xmake g --pkg_searchdirs=<download-dir>` 并根据报错提示，手动下载软件包并重命名为指定名字
  
    + 一般情况，下载的软件包不用改名就可以识别到。一种方便的方式是将浏览器的默认下载路径加入搜索路径，例如如果在 Windows 上使用 Edge 浏览器，则默认下载路径为 `C:\Users\<username>\Downloads`，设置

      ```shell
      xmake g --pkg_searchdirs=C:\Users\<username>\Downloads
      ```

      这样在 VSCode 自带终端等比较智能的终端中，按住 Ctrl+单击报错信息提示的下载url 跳转到浏览器下载，等一小会重新运行 xmake 即可。这种方法适用于使用浏览器插件作为代理或者不喜欢每次在命令行进行设置的同学。
  
- Q. 在命令行中输入 `xmake` 报错找不到编译器或「cannot get program for cxx」怎么办？？

- A. xmake 在各平台会默认使用该平台原生工具链，例如 Windows 上的 Visual Studio，MacOS 上的 XCode，Linux 上的 GCC，而对其他的工具链会报错找不到编译器。如果要使用 Msys2 提供的GCC编译器，在 Windows 上运行：
  ```shell
  xmake f -p mingw
  ```
  再执行编译步骤。如果要使用 Clang 编译器，运行
  ```shell
  xmake f --toolchain=clang
  ```
  再执行编译步骤。

- Q. 在 Windows 的 powershell 中输入 `xmake`，提示找不到 Visual Studio，但是我明明安装了怎么办？

- A. 首先确保 Visual Studio 的安装路径不含中文，如果含中文需要重新安装到其他路径。其次打开 Visual Studio Installer，找到 `使用 C++ 的桌面开发` ，在可选项中查看 `对 v1xx 生成工具(最新)的 C++/CLI 支持` 一项是否勾选，如果未勾选，安装这一项并重新尝试编译。

- Q. 我的系统上安装了多个编译器，怎样指定使用哪一个编译器编译？

- A. xmake 对编译器提供了全局缓存。如果你新安装了一个编译器，需要使用`xmake g -c`清理全局缓存，再在项目目录运行`xmake f -c`重新探测编译器。对不同的编译器，有不同指定版本的方式。例如，指定使用 Visual Studio 2022：
  ```shell
  xmake f --vs=2022
  ```
  指定使用gcc11：
  ```shell
  xmake f --toolchain=gcc-11
  ```

- Q. 我使用 Mac OS 系统，安装时报错 `invalid Darwin version number: macos 12.3`

- A. 使用的 XCode 版本过低，将 XCode 更新到最新版本即可。

### 写在后面

到这里，你应该已经能够编译运行我们提供的 Lab 代码，并能够在舒适的开发环境中进行开发了。遇到问题的同学请尽快向助教提出你的疑问。

本次 lab0 中，大家只需要完成上述环境的配置即可。本文档的后半部分是一个简单导引，帮助大家理解示例代码，介绍其中需要模仿的关键模块；在后续的 lab 中，同学们可以搬运 lab0 的可视化、交互部分代码并进行自己的修改。大家可以提前阅读并熟悉我们的代码框架，或者在以后写 lab 的时候再进行参考。写过可视计算与交互概论课程大作业的同学可以跳过这部分内容。

## Part 2: A Tour of Our Codebase Structure

本课程的代码框架继承自可视计算与交互概论课程并做了微调，[这篇指南](https://vcl.pku.edu.cn/course/vci/labs/engine.pdf)是对该代码框架的详细介绍，以下是一个简要总结。

### How to Complete Your Code for Physics Simulation

在每次作业中，大家需要像 lab0 给出的效果那样，搭建若干模拟场景（即左上角的5个 cases），并提供键鼠交互或是通过按键修改参数，给出物理真实的模拟效果。为此，需要完成三个层次的构建：

- 在模拟层面，需要大家编写物理模拟算法，模拟场景中物体的运动；
- 在交互层面，需要创建ImGui交互方式并与模拟场景进行绑定；
- 在渲染层面，需要将物理场景渲染成图片，提供丰富且便捷的可视化效果。

我们的代码框架为大家提供了大量封装，并完成了 OpenGL 渲染的相应初始化、管线搭建、ImGui 窗口创建等繁杂的工程，大家只需完成下列操作，即可达到课程要求并生成赏心悦目的模拟 demo：

1. 编写物理模拟算法：建议大家将其编写成一个类，存储场景中的物理信息（位置、速度等），并提供基于上一帧结果，计算下一帧物理量的接口；
2. 为每个模拟场景编写 `CaseXXX` 类，完成物理场景与交互的耦合，它需要：
    - 是 `Common::ICase` 的派生类，从而能被代码框架的其他接口所调用；
    - 存储一个物理模拟算法的实例，从而能够访问并控制场景中的物理量，进行渲染与交互；
    - 实现 `OnProcessInput` 函数，控制如何进行鼠标交互；
    - 实现 `OnSetupPropsUI` 函数，控制左侧交互栏的种类，并将其与场景中物理量进行绑定；
    - 实现 `OnRender` 函数，控制每一帧时场景应如何渲染在屏幕上；
3. 为整个应用编写一个`App`类，它整合所有的 cases，需要：
    - 是 `Engine::IApp` 的派生类；
    - 存储一个`Common::UI`的实例`_ui`，并为每个 cases类存储一个实例；
    - 编写`OnFrame`函数，执行 `_ui.Setup` 指令，即可完成 ImGui 窗口的创建
    - 在 `main` 函数中，执行 `RunApp` 函数，函数模版设为刚刚编写的`App`类，即可完成外部渲染流程的创建，

行文至此，大家可能还是觉得无从下手；不过，大家需要的许多操作与函数实现都可以照搬本次给出的样例代码！下面，我将以 lab0 示例代码的 CaseMassSpring 为例，介绍样例代码是如何达到上述效果的。

### An Example Case: CaseMassSpring

（以下文件均位于 `src/VCX/Labs/0-GettingStarted` 目录下）
`main.cpp`, `App.h`, `App.cpp` 中的写法可以直接照搬，只需将 `RunApp` 函数调用时的模版类改成自己定义的 `App` 类，在自己的 `App` 类中存自己的 `CaseXXX` 实例即可。这样，`RunApp` 函数会完成渲染流程的搭建，`_ui.Setup` 函数会完成ImGui窗口的初始化。

`CaseMassSpring` 类中存储的成员变量的作用是：
- 与物理过程有关的变量，包括 `_massSpringSystem` 控制物理系统的模拟以及物理参数的设定；`_stopped` 标志模拟是否停止
- 可通过交互改变的渲染参数，如` _particleSize` 等 ；此外，相机 `_camera` 的位置可以通过交互改变，由 `_cameraManager` 提供的接口负责；此外，`_cameraManager` 也提供了键鼠交互的接口。
- 与渲染有关的变量：其中，`_frame` 代表当前渲染的帧，`_program` 控制渲染所需的着色器。`_particlesItem` 与`_springsItem` 分别负责点与弹簧的渲染，前者只需要渲染单独的点，因此属于 `UniqueRenderItem` 的实例；后者还需记录顶点间的连接关系（线段，三角面片都属于此类），因此属于 `UniqueIndexedRenderItem` 的实例。在初始化时，我们需要设置正确的绘制类型 `PrimitiveType::Points/Lines`.

`OnSetupPropsUI` 函数的内容比较直接，通过为 ImGui 函数绑定相应的指针来操控相应的参数。如果想实现的效果在样例代码中没有涉及，大家可以浏览 [vcx](https://gitee.com/pku-vcl/vcx2024/tree/master/) 代码库的各个 branch 或在网上进行搜索。

`OnProcessInput` 函数中，我们简单的调用了 `_cameraManager` 的交互函数，达到鼠标左键旋转/右键及 WASDQE 按键平移/滚轮沿连线方向平移相机位置的效果；

`OnRender` 函数中，我们首先 **根据当前帧与前一帧的时间间隔，计算仿真结果**。

```c++
if (! _stopped) _massSpringSystem.AdvanceMassSpringSystem(Engine::GetDeltaTime());
```

这是渲染和仿真效果耦合的核心，推荐同学们在后续的lab中，也按照该思路编写相应函数接口。后续的渲染流程比较繁杂，包括的步骤为：
- 设定当前帧的长宽，更新相机位置；
- 对于 `_particlesItem` ，渲染时需要更新其管理的顶点位置（使用 `UpdateVertexBuffer` 函数）；对于 `_springsItem` ，还额外需要顶点间的连接关系（使用 `UpdateElementBuffer` 函数，由于模拟过程中连接关系不变，因此只在 `ResetSystem` 中调用）
- 渲染开始时，需要做一些设置：启用当前帧，调用 `glEnable(GL_LINE_SMOOTH)` 为所画的线提供抗锯齿，设置点的大小以及线宽；
- 接下来就是使用着色器分别为点线完成渲染。渲染前，需要先为着色器传递参数，包括相机参数 `u_Projection` 与 `u_View`，以及绘制的颜色 `u_Color`，随后调用 `Draw` 函数进行渲染；
- 渲染完成后，恢复相应设置后函数返回。

以上我们简单介绍了大家需要完成的接口，希望通过上述讲解，大家能熟悉交互以及渲染需要完成的各步骤。以下是对一些环节的补充。

### More on Rendering

- 在 lab1 中，我们需要涉及到刚体的模拟，立方体的渲染可以参考 CaseBox 中的渲染流程。样例代码中单独渲染了立方体盒以及框架，并且调用 `glEnable(GL_DEPTH_TEST)` 开启深度测试。
- 在 lab2 中我们需要完成粒子法流体模拟，样例代码中 CaseFluid 写了一份示例的渲染代码。我们将流体粒子渲染成小球，使用另一套着色器；该着色器的部分参数由 `BindUniformBlock` 函数与 `_sceneObject` 中的 `PassConstantsBlock` 相绑定。渲染时，将多个小球合并成一个 `ModelObject` 实例，再进行渲染。
- 在 lab3 中我们需要完成FEM模拟，对于软体内部的离散结构，大家可以选择像弹簧质点那样渲染各四面体的顶点和边（可以考虑是否要开启深度测试），或者渲染外层 mesh.

### More on Interations

在 CaseMassSpring 代码中，我们只是简单地调用了代码库中相机交互的方法。如果大家希望设计更丰富的交互方法，可以关注这几处的代码：

- 想要设计自己的键鼠交互的同学，可以参考 `OrbitCameraManager` 类中 `ProcessInput` 的写法，调用ImGui的相关接口判断键鼠状态。
- 在CaseBox中，我们设计了 `OnProcessMouseControl` 函数，它在 `OnRender` 的开头调用来改变场景，使得按住alt键后鼠标左键可以平移物体位置。大家可以关注一下 `OrbitCameraManager` 类中 `ProcessInput` 的写法中关于 `altKey` 的部分，它控制是要平移相机还是预计算出物体在世界坐标下应该平移的距离，再在 `getMouseMove` 函数调用时返回这一值。

```c++
if (movingByMouse) {
    if (ScreenSpacePanning) {
        _moveDelta -= q * glm::vec3(panLeft, panUp, 0.f) + panFront * front;
    } else {
        _moveDelta -= q * glm::vec3(panLeft, 0.f, 0.f) + panUp * camera.Up + panFront * front;
    }
    _state |= StateMove;
}
```

```c++
if (! altKey) {
    if (rotating) {
        _spDelta.Theta -= delta.x * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
        _spDelta.Phi -= delta.y * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
        _state |= StateRotate;
    }
    if (wheeling) {
        _logScale += ZoomSpeed * wheel;
        _state |= StateDolly;
    }
}
```

大家在设计时，也可以用类似的方法修改对应的函数，来设计自己的交互方法。

## 写在最后

如果想要进一步了解 Lab 代码可以阅读[这篇指南](https://vcl.pku.edu.cn/course/vci/labs/engine.pdf)，也可以向助教提出你的疑问。

衷心欢迎大家对我们的课程和 Lab 设计提出自己的看法，\*也许明年的 Lab 代码库就是你的作品哦\*。

最后，这门课程由北京大学可视计算与学习实验室独家荣誉出品，这里有最前沿的可视计算研究，最宽松的学习氛围，最 nice 的学长学姐，欢迎大家来玩！详情请咨询课程助教~