# GitHub 协作指南

> 本项目地址：https://github.com/geyutong985-eng/arduino-project

---

## 目录

- [一、协作流程](#一协作流程)
- [二、分支协作](#二分支协作)
- [三、常见问题](#三常见问题)

---

## 一、协作流程

### 核心流程

```
创建分支 → 写代码 → 提交 → 推送 → 创建 PR → 组长合并
```

### 基本规则

1. **必须使用分支**：每人一个分支，不要直接在 main 上改
2. **写完就推**：不要等到最后一天才提交，容易出问题
3. **每天先 pull 再 push**：每天开始前先拉最新代码，减少冲突
4. **写清楚改动说明**：commit message 要让人看懂你改了啥
5. **提交 PR 后等待合并**：不要自己点合并，等组长审核

---

## 二、分支协作

### 2.1 第一次使用：克隆项目

如果你本地还没有项目，先下载到本地：

**Mac：**
```bash
cd ~/Desktop
git clone https://github.com/geyutong985-eng/arduino-project.git
cd arduino-project
```

**Windows：**
```bash
cd %USERPROFILE%\Desktop
git clone https://github.com/geyutong985-eng/arduino-project.git
cd arduino-project
```

> **说明**：
> - `cd` = 进入某个文件夹
> - `git clone` = 把 GitHub 上的项目下载到本地
> - `~/Desktop` = 你电脑桌面的路径（Mac 用，`~` 代表你的用户文件夹）
> - `%USERPROFILE%\Desktop` = Windows 桌面路径

---

### 2.2 创建自己的分支

**Mac 和 Windows 通用：**
```bash
git checkout -b 你的名字
```

> **说明**：
> - `git checkout -b` = 创建并切换到新分支
> - `你的名字` = 用你的名字或拼音命名，如 `zhangsan`、`lisi`
> - 每人只能用这一个分支

---

### 2.3 每天的工作流程

**每天开始时（Mac）：**
```bash
cd ~/Desktop/arduino-project
git pull
git checkout 你的分支名
```

**每天开始时（Windows）：**
```bash
cd %USERPROFILE%\Desktop\arduino-project
git pull
git checkout 你的分支名
```

> **说明**：
> - `git pull` = 从 GitHub 下载最新代码
> - `git checkout 你的分支名` = 切换到你自己的分支

---

**写完代码后：**
```bash
git add .
git commit -m "完成了xxx功能"
git push
```

> **说明**：
> - `git add .` = 把所有修改的文件标记为"准备好要提交"
> - `git commit -m "说明"` = 正式提交修改，"说明"写你改了啥
> - `git push` = 把提交发送到 GitHub

---

### 2.4 创建 PR（请求合并）

**网页操作：**
1. 打开 https://github.com/geyutong985-eng/arduino-project
2. 会看到黄色提示 "xxx branch has recent pushes"
3. 点击 **Compare & pull request**
4. 填写标题（如 "完成了温度传感器功能"）
5. 点击 **Create pull request**

> **说明**：PR 就是请求把你的分支合并到 main 分支，需要组长同意

---

### 2.5 组长审核合并

1. 收到 PR 后，组长查看代码改动
2. 没问题的话点击 **Merge pull request**
3. 点击 **Confirm merge** 完成合并
4. 可以删除已合并的分支（可选）

---

### 常用命令速查

| 命令 | 做什么 |
|------|--------|
| `git branch` | 查看当前在哪个分支 |
| `git checkout -b 名字` | 创建并切换到新分支 |
| `git checkout 名字` | 切换到某个分支 |
| `git status` | 查看有哪些文件改了 |
| `git add .` | 把所有修改标记为待提交 |
| `git commit -m "说明"` | 提交修改 |
| `git push` | 推送到 GitHub |
| `git pull` | 从 GitHub 拉最新代码 |
| `git checkout -- .` | 恢复误删的文件 |

---

## 三、常见问题（让AI帮你！！！）

### 推送时报错 "permission denied"

可能没登录，先运行：
```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

### 推送时报错 "Authentication failed"

需要重新认证，联系组长帮忙。

### pull 的时候有冲突

先别慌，把错误截图发群里，我来帮你解决。

### 不小心删了本地文件

运行 `git checkout -- .` 可以恢复所有文件。

---

有问题找AI！
