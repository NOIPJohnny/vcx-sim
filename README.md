# VCX-sim

本仓库为北京大学2026年图形学物理仿真课程的作业代码，*仅留作个人使用，若有侵权请联系删除*。


## 项目构建
请确保已安装xmake构建工具。
使用以下命令构建项目：

```bash
xmake
```
构建完成后，按照需求运行对应的可执行文件：
- 刚体仿真：`lab1`
- 流体仿真：`lab2`
- 软体仿真：`lab3`
- 耦合仿真：`lab4`，其中包含刚体流体耦合和流体软体耦合两部分

```bash
xmake run lab1
xmake run lab2
xmake run lab3
xmake run lab4
```