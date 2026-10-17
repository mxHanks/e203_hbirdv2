# E203 SoC Integration for ChipContest

本仓库基于开源 [Hummingbirdv2 E203](https://github.com/riscv-mcu/e203_hbirdv2)
修改，用作 [ChipContest](https://github.com/mxhanks/ChipContest) AI 加速器项目的 RISC-V SoC 硬件平台。

> 本仓库是上游 E203 的 Fork，不是独立重新实现的 CPU。原始项目说明、版权及许可证信息见下方官方 README。

## 本仓库的用途

我们使用 E203 作为 SoC 平台，主要进行：

- RISC-V CPU 与 Chisel AI 加速器的集成。
- ICB 总线外设接口开发。
- MMIO 寄存器及软件控制协议设计。
- 加速器与片上存储器的数据传输。
- SoC RTL 仿真、软件协同验证与后续 PPA 评估。

加速器的 Chisel 源码位于 **ChipContest 主仓库**，本仓库主要管理与 E203 相关的 SoC RTL 修改。

## 当前集成状态

| 项目                           | 状态   |
|--------------------------------|--------|
| E203 原版 SoC 仿真             | 已验证 |
| ChipContest Git Submodule 管理 | 已配置 |
| ICB o15 预留接口定位           | 已确认 |
| 自定义 ICB 从设备              | 待完成 |
| CPU 通过 MMIO 读写自定义寄存器 | 待验证 |
| Chisel 加速器 RTL 接入         | 待完成 |
| SoC 端到端矩阵计算             | 待完成 |

### 计划中的 ICB 接口

我们准备使用 E203 外设总线的 `o15` 端口，暂定地址区域：

```text
Base Address: 0x10042000
Region Size:  4 KiB（计划）
Interface:    ICB
```

计划新增独立的 `accel_icb_slave` 模块，负责接收 CPU 发来的 ICB 请求，并将其转化为加速器控制操作。

当前地址和寄存器映射属于实验设计，尚未作为已验证的接口协议发布。

## 本地仿真

建议从 ChipContest 主仓库克隆并初始化 Submodule，以确保 E203 版本一致。

在主仓库根目录：

```bash
git submodule update --init --recursive
```

随后进入本仓库：

```bash
cd third-party/e203_hbirdv2/vsim
```

构建基础仿真环境：

```bash
make clean
make install
make compile SIM=iverilog
make run_test SIM=iverilog DUMPWAVE=0
```

说明：

- `make install`：将 RTL 和 Testbench 复制到仿真工作目录。
- `make compile`：编译 SoC 仿真程序。
- `make run_test`：执行测试程序。
- `DUMPWAVE=0`：关闭波形转储。

修改 `rtl/e203/` 下的源文件后，需要确保 `vsim/install/rtl/` 中用于仿真的代码已经更新，避免使用过期副本。

## 目录说明

```text
e203_hbirdv2/
├── rtl/e203/
│   ├── core/          # CPU 核心
│   ├── subsys/        # SoC 子系统及外设总线
│   └── perips/        # 外设相关 RTL
├── tb/                # 仿真 Testbench
├── vsim/              # RTL 仿真工程
├── riscv-tools/       # RISC-V 测试工具
├── fpga/              # FPGA 工程
└── doc/               # 官方文档
```

当前集成工作的重点文件是：

```text
rtl/e203/subsys/e203_subsys_perips.v
```

此文件负责部分外设总线连接和地址译码。后续自定义外设尽量独立成模块，避免在官方 SoC 文件中混入大量加速器业务逻辑。

## Git 协作

本仓库具有独立的 Git 历史。

- `origin`：团队维护的 E203 Fork。
- `upstream`：官方 `riscv-mcu/e203_hbirdv2` 仓库。
- SoC 修改应在独立功能分支开发。
- 不直接在官方上游仓库提交我们的比赛专用修改。

推荐工作流程：

```bash
# 在 E203 子模块内部
git status
git add rtl/e203/
git commit -m "feat: add accelerator ICB peripheral"
git push -u origin HEAD
```

然后回到 ChipContest 主仓库：

```bash
cd ../..
git add third-party/e203_hbirdv2
git commit -m "chore: update E203 submodule"
git push
```

**必须先推送 E203 的新 Commit，再提交主仓库中的 Submodule 引用。**

## 上游项目与许可证

本项目基于 Nuclei 开源的 Hummingbirdv2 E203。

- [上游仓库](https://github.com/riscv-mcu/e203_hbirdv2)
- [官方文档](https://doc.nucleisys.com/hbirdv2/)
- [ChipContest 主仓库](https://github.com/mxhanks/ChipContest)

保留原始版权声明及 `LICENSE`，任何第三方模块仍遵循其各自许可证。

---

# Original Hummingbirdv2 Documentation

以下保留上游项目原有的 README 内容及相关说明。

Hummingbirdv2 E203 Core and SoC
===============================

[![Deploy Documentation](https://github.com/riscv-mcu/e203_hbirdv2/workflows/Deploy%20Documentation/badge.svg)](https://doc.nucleisys.com/hbirdv2)

> [!NOTE]
> **Hummingbird E603** is now available —— a 64-bit RISC-V core you can
freely use for academic and non-commercial projects.  
> Explore it here: [Nuclei-Software/e603_hbird](https://github.com/Nuclei-Software/e603_hbird)

About
-----

This repository hosts the project for open-source Hummingbirdv2 E203 RISC-V processor Core and SoC, it's developped and
opensourced by [Nuclei System Technology](www.nucleisys.com), the leading RISC-V IP and Solution company based on China
Mainland.

This's an upgraded version of the project Hummingbird E203 maintained
in [SI-RISCV/e200_opensource](https://github.com/SI-RISCV/e200_opensource), so we call it Hummingbirdv2 E203, and its
architecture is shown in the figure below.
![hbirdv2](pics/hbirdv2_soc.JPG)

In this new version, we have following updates.

* Add NICE (Nuclei Instruction Co-unit Extension) for E203 core, so user could create customized HW co-units with E203
  core easily.
* Integrate the APB interface peripherals (GPIO, I2C, UART, SPI, PWM)
  from [PULP Platform](https://github.com/pulp-platform) into Hummingbirdv2 SoC, these peripherals are implemented in
  Verilog language, so it's easy for user to understand.
* Add new development boards (Nuclei ddr200t and mcu200t) support for Hummingbirdv2 SoC.

**Welcome to visit https://github.com/riscv-mcu/hbird-sdk/ to use software development kit for the Hummingbird E203.**

**Welcome to visit https://www.rvmcu.com/community.html to participate in the discussion of the Hummingbird E203.**

**Welcome to visit http://www.rvmcu.com/ for more comprehensive information of availiable RISC-V MCU chips and embedded
development.**


Detailed Introduction and Quick Start-up
----------------------------------------

We have provided very detailed introduction and quick start-up documents to help you ramping it up.

The detailed introduction and the quick start documentation can be seen
from https://doc.nucleisys.com/hbirdv2/.

By following the guidences from the doc, you can very easily start to use Hummingbirdv2 E203 processor Core and SoC.

What are you waiting for? Try it out now!

Dedicated FPGA-Boards and JTAG-Debugger
---------------------------------------

In order to easy user to study RISC-V in a quick and easy way, we have made dedicated FPGA-Boards and JTAG-Debugger.

#### Nuclei ddr200t development board

<img src="pics/DDR200T.JPG" width= 80% alt="DDR200T"/>

#### Nuclei mcu200t development board

<img src="pics/MCU200T.JPG" width= 80% alt="MCU200T"/>

#### Hummingbird Debugger

![Debugger](pics/debugger.JPG)

The detailed introduction and the relevant documentation can be seen from https://nucleisys.com/developboard.php.

HummingBird SDK
---------------

Click https://github.com/riscv-mcu/hbird-sdk for software development kit.

WeChat Group
------------

If you would like to join our WeChat group for discussion and support,
please scan the following QR code:

![wechat QR code](./pics/QR_code.png)

Release History
---------------

#### Notice

* **Many people asked if this core and SoC can be commercially used, the answer as below:**
    - According to the Apache 2.0 license, this open-sourced core can be used in commercial way.
    - But the feature is not full.
    - The main purpose of this open-sourced core is to be used by students/university/research/
      and entry-level-beginners, hence, the commercial quality (bug-free) and service of this core
      is not not not warranted!!!

#### Release 0.2.1, Feb 26, 2021

This is `release 0.2.1` of Hummingbirdv2.

+ Hbirdv2 SoC
    - Covert the peripheral IPs implemented in system verilog to verilog implementation.

+ SIM
    - Add new simulation tool (iVerilog) and wave viewer (GTKWave) support for Hummingbirdv2 SoC

#### Release 0.1.2, Nov 20, 2020

This is `release 0.1.2` of Hummingbirdv2.

+ Hbirdv2 SoC
    - Remove unused module
    - Add one more UART

+ FPGA
    - Add new development board (Nuclei mcu200t) support for Hummingbirdv2 SoC

#### Release 0.1.1, Jul 28, 2020

This is `release 0.1.1` of Hummingbirdv2.

NOTE:

+ This's an upgraded version of the project Hummingbird E203 maintained in
  [SI-RISCV/e200_opensource](https://github.com/SI-RISCV/e200_opensource).
+ Here are the new features of this release.
    - Add NICE (Nuclei Instruction Co-unit Extension) for E203 core
    - Integrate the APB interface peripherals (GPIO, I2C, UART, SPI, PWM) from PULP Platform
    - Add new development board (Nuclei ddr200t) support for Hummingbirdv2 SoC. 
