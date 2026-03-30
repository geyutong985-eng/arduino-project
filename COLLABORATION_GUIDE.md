# GitHub 协作指南

> 本项目地址：https://github.com/geyutong985-eng/arduino-project

---

## 一、三种方式对比

| 方式 | 难度 | 适合谁 | 特点 |
|------|------|--------|------|
| **GitHub 网页** | 最简单 | 不懂命令的人 | 鼠标操作，看不到本地文件 |
| **GitHub CLI (gh)** | 简单 | 稍懂一点命令行 | 几个简单命令，比网页快 |
| **Git 终端命令** | 稍难 | 愿意学的人 | 最完整，能处理所有情况 |

**推荐**：网页打底，命令进阶。如果终端有 AI 助手（Cursor、Copilot 等），可以直接让它帮你写命令。

---

## 二、网页操作（最简单）

### 2.1 查看和下载代码

1. 打开 https://github.com/geyutong985-eng/arduino-project
2. 点 **Code → Download ZIP** 下载整个项目

### 2.2 在网页上编辑文件

1. 进入仓库，点进任意一个文件
2. 点右上角 **编辑图标（铅笔）**
3. 改完后点 **Commit changes**（记得写改动说明）
4. ⚠️ 网页改完后，本地文件夹不会自动更新，需要手动同步

### 2.3 在网页上新建文件

1. 点 **Add file → Create new file**
2. 写文件名（如 `sensor.ino`）和内容
3. 点 **Commit changes**

### 2.4 同步网页最新内容到本地文件夹

如果网页有人改了代码，你的本地文件夹不会自动更新。

**方法**：重新下载 ZIP 覆盖本地，或者用命令同步（见下方命令部分）。

---

## 三、GitHub CLI（稍简单）

### 3.1 安装

Mac 上打开终端，运行：

```bash
brew install gh
```

然后登录：

```bash
gh auth login
```

按提示选择就好。

### 3.2 克隆项目到本地

第一次下载项目：

```bash
git clone https://github.com/geyutong985-eng/arduino-project.git
```

会下载到当前文件夹。

### 3.3 更新本地代码

别人改了 GitHub 上的代码，你想同步到本地：

```bash
cd arduino-project
git pull
```

### 3.4 推送你的代码到 GitHub

```bash
cd arduino-project
git add .
git commit -m "你的改动说明"
git push
```

### 3.5 如果推送失败

可能是因为 GitHub 上有新的改动，你先拉下来：

```bash
git pull
# 如果有冲突，联系我帮你解决
git push
```

---

## 四、Git 终端命令（最完整）

> 如果你用 AI 助手（Cursor、Copilot 等），可以直接让它帮你写命令，比如："帮我执行 git add . git commit 和 git push"

### 4.1 首次设置（只用做一次）

打开终端，运行：

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

### 4.2 克隆项目（第一次用）

```bash
git clone https://github.com/geyutong985-eng/arduino-project.git
cd arduino-project
```

### 4.3 每天写完代码后提交

```bash
# 进入项目文件夹
cd ~/Desktop/arduino-project

# 查看状态（可选）
git status

# 把所有改动加入暂存区
git add .

# 提交（把改动存档）
git commit -m "这里写你这次改了啥"

# 推送到 GitHub
git push
```

### 4.4 每天开始前同步最新代码

```bash
cd ~/Desktop/arduino-project
git pull
```

### 4.5 常用命令速查

| 场景 | 命令 |
|------|------|
| 查看有哪些改动 | `git status` |
| 查看提交记录 | `git log` |
| 看某个文件改了啥 | `git diff 文件名.ino` |
| 撤销某个文件的改动 | `git checkout -- 文件名.ino` |

---

## 五、AI 助手帮你用 Git

如果你用 Cursor、Copilot 等有 AI 的终端，可以直接让它帮你干活：

**示例对话：**
- "帮我把今天的改动提交并推送到 GitHub"
- "有人在 GitHub 上改了代码，帮我同步到本地"
- "我想创建一个新分支写电机代码，怎么做？"

AI 会帮你写好命令，你只需要复制粘贴跑就行。

---

## 六、常见问题

**Q：推送时报错 "permission denied"**
A：可能没登录，先运行 `gh auth login` 或 `git config` 设置身份

**Q：推送时报错 "fatal: Authentication failed"**
A：需要重新认证，联系组长帮忙

**Q：pull 的时候有冲突怎么办？**
A：先别慌，把错误截图发群里，我来帮你解决

**Q：我不小心删了本地文件，能恢复吗？**
A：可以，运行 `git checkout -- .` 恢复所有文件

---

## 七、注意事项

1. **不要直接改 main 分支上的代码**，除非你是负责人
2. **commit 前先 add**，不然不会提交上去
3. **commit message 要写清楚**，方便大家知道改了什么
4. **常用 `git pull`** 开始前同步最新代码，减少冲突

---

有问题找组长！