# Isaac Sim 搭建基础指南

- 官方教程：https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements
- Nvidia驱动：https://www.nvidia.cn/drivers/lookup/

### 0 硬件要求

Isaac Sim 对硬件要求很高，具体如下：

| Element | Minimum Spec                              | Good                                      | Ideal                                                        |
| ------- | ----------------------------------------- | ----------------------------------------- | ------------------------------------------------------------ |
| OS      | Ubuntu 20.04/22.04  Windows 10/11          | Ubuntu 20.04/22.04  Windows 10/11          | Ubuntu 20.04/22.04  Windows 10/11                             |
| CPU     | Intel Core i7 (7th Generation)  AMD Ryzen 5 | Intel Core i7 (9th Generation)  AMD Ryzen 7 | Intel Core i9, X-series or higher  AMD Ryzen 9, Threadripper or higher |
| Cores   | 4                                         | 8                                         | 16                                                           |
| RAM     | 32GB*                                     | 64GB*                                     | 64GB*                                                        |
| Storage | 50GB SSD                                  | 500GB SSD                                 | 1TB NVMe SSD                                                 |
| GPU     | GeForce RTX 3070                          | GeForce RTX 4080                          | RTX Ada 6000                                                 |
| VRAM    | 8GB*                                      | 16GB*                                     | 48GB*                                                        |

### 1 安装 Omniverse Launch

##### 下载Omniverse Launch

通过以下链接下载Omniverse Launch：https://www.nvidia.cn/omniverse/download/

填写表单，获取 windows 以及 Linux 端下载链接，按需下载。下载完成点击打开正常安装即可。

Linux 端 Omniverse 也可在终端通过命令行安装打开：

```bash
# Install fuse
sudo apt install fuse -y  
# Install wget
sudo apt install wget -y
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Granting execution permission
sudo chmod +x omniverse-launcher-linux.AppImage
# Run Omniverse Launcher
./omniverse-launcher-linux.AppImage
```

##### 注册/登录 Nvidia 账号

Omniverse 启动需要 Nvidia 账号。按照指引注册、登陆即可。可注意下安装路径：

| 路径     | 英文         | 作用                                                         |
| -------- | ------------ | ------------------------------------------------------------ |
| 库路径   | library path | 此位置指定在Omniverse启动器里安装的应用程序、连接器和工具的安装位置，比如Isaac Sim。 |
| 数据路径 | data path    | 数据路径是Omniverse Nucleus服务器（local版本）存储所有内容的位置。 |
| 资源路径 | content path | 资源路径是Omniverse下载一些资产到本地时默认的下载位置。      |
| 缓存路径 | cache path   | 缓存数据库（Omniverse Cache）用于存储其缓存文件的位置（如果要安装Omniverse Cache的话）。 |

### 2 安装 Isaac-sim

##### 配置 Nucleus

Nucleus 用于 Omniverse 资产管理。在访问控制的前提下，Omniverse 客户端可以将数字资产和虚拟世界的修改**发布**到Nucleus数据库（DB），或**订阅**其变化。更改在连接的应用程序之间实时传输。数字资产可以包括描述虚拟世界及其随时间演变的几何形状、灯光、材料、纹理和其他数据。

在 **EXCHANGE** 部分，搜索 **Nucleus Navigator**，下载：

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6a6a5d96e32542fa94054c887b87d4ab.png)

**注意事项**

- Omniverse launch 中下载资源需要挂梯子；
- 如果挂梯子还是下载失败，可指定程序以梯子代理的端口运行（以 win 为例，在 Omniverse Launch 应用图标中`属性`->`target`内容后添加`--proxy-server={serverAddress}:{port}`；

![_images/launcher_setup_proxy-flags.png](https://docs.omniverse.nvidia.com/launcher/latest/_images/launcher_setup_proxy-flags.png)

- 如果遇到下载中断，可以先暂停下载，然后退出 Omniverse launch，在重新打开下载；

完成 Nucleus Navigator 下载后，打开 Nucleus Navigator，配置本地服务：

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/5921bee956a84df9953ddce78e4e0e96.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6abbf30493264fc0905393d4bd3c80ed.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/0035579258fc47ff9c70d4191245a1da.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6af19638fd5c4ec091e3a5d8204b277b.png)

##### 下载 Isaac-sim

进入 Omniverse 的 **EXCHANGE** 页面，选择 ISAAC SIM，选择`其他发行版本`-> `4.00版本`，点击**install**，等待自动下载安装完成。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/41e8c8742e4e48dab37a649a18939e49.png)

##### 配置 Isaac-sim Python 环境

这里提供创建isaac-sim conda 环境的方法。

进入 isaac-sim 下载文件夹（见安装 Omniverse 时指定的 Library Path），找到 `environment.yml` 文件，在当前目录 conda 终端运行：

```bash
conda env create -f environment.yml
conda activate isaac-sim
```

### 3 SimReady 3D Assets

[SimReady](https://developer.nvidia.com/omniverse/simready-assets) 为官方提供的 3D 资产数据集，OpenUSD 格式，支持在 Omniverse 中使用。

SimReady 下载方式与 Isaac Sim 类似，在Omniverse Launch里下载，具体名称为 Isaac Sim Assets Pack，共四个包：

![img](https://img-blog.csdnimg.cn/direct/0d3d8bfbc1204d2a9881ed8266afcbfb.png)

- 以下网盘链接可获取资产

  链接：https://pan.quark.cn/s/a8ff48afa378

  提取码：yE2V

### 4 简单使用 Isaac-sim

在 **LIBRARY** 中找到 Isaac Sim，点击 **Launch** 启动，按照默认选项，点击 **START** 继续启动：

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4d58c42b124448d3b8c4ed97e26e706c.png)

通过 GUI 操作导入机器人 URDF 模型：

参考教程：[Isaac Sim 仿真机器人urdf文件导入_cassie urdf文件-CSDN博客](https://blog.csdn.net/hai411741962/article/details/135192955)

**一些细节**：

- URDF 文件一般放在库目录中 `isaac-sim-4.0.0/extscahe/omni.impoter.urdf-1.14.1+106.0.0.wx64.r.cp310/data/urdf` 目录下；
- 实例脚本代码以及相关文档可在库目录中 `isaac-sim-4.0.0` 中找到；
- 官方文档链接：[What Is Isaac Sim? — Omniverse IsaacSim latest documentation (nvidia.com)](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
