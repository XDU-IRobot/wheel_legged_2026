# 2026 赛季嵌入式组软件仓库2（达妙H7部分）

## Branch status

| 模块          | 编译测试状态                                                                                                                                                                                                           |
|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| test target | [![build-test_target](https://github.com/XDU-IRobot/embedded-2026-h7/actions/workflows/build-test_target.yml/badge.svg)](https://github.com/XDU-IRobot/embedded-2026-h7/actions/workflows/build-test_target.yml) |

## Software packages

- CubeMX 6.17.0

- STM32Cube FW_H7 V1.13.0

## [开发板资料 `docs/dm-mc02`](docs/dm-mc02)

## 工作流程 ***第一次开发先看这里！***

1. `git pull`、`git merge main`，确保自己的分支有 `main` 分支最新的内容。

2. 阶段性完成工作，自己的分支稳定下来之后，立刻发起 Pull Request 合并回 `main` 分支。

3. **如果需要改动CubeMX里的设置，调整硬件资源，在 [这里](docs/cubemx_changelog.md) 和 [这里](docs/hw_resource.md)
   记录改动的内容和原因。（务必自觉填写！方便自己方便他人。）**

### 目录结构

开发**_严格执行_**非侵入式原则，在 CubeMX 生成代码的基础上，所有用户代码均放置在 `app/` 目录下，对 CubeMX 生成的代码仅做最小化的改动：在
`main()` 函数里调用用户程序的入口点 `AppMain()`。

`app/` 目录包含 `common/` 和 `targets/` 两个子目录：

- `common/`: 存放通用代码模块，这些模块会被所有目标共享使用。例如：“二轴云台控制器”、“定时任务执行器”这种所有兵种开发时都会用的模块，应放在此目录下。
- `targets/`: 存放特定“目标”的代码，每一块 C 板控制的一个实体称作一个目标。例如：“老舵轮的底盘”、“新串腿底盘”、“某架无人机云台”称作一个目标。每个目标都有自己独立的代码目录，目录名即为目标名。

### 分支规则

- `main` ：主分支，始终保持可编译、可运行状态。已设置保护规则，禁止直接 push。

- `target/<目标名>`： 每个目标对应一个分支。例如 `target/steer_infantry_cs`。大多数情况下每个目标都由一个人负责，如果有例外情况，要多人合作开发一辆车，自行协调分支规则。

### FAQ

- ### `git submodule update --init --recursive` 子模块！！不会就学！！https://www.runoob.com/git/git-submodule.html

- ### [指南 1：添加新模块](app/common/new_common_module.md)

- ### [指南 2：添加新目标](app/targets/new_target.md)

- ### [指南 3：OpenOCD 调试配置](openocd/README.md)