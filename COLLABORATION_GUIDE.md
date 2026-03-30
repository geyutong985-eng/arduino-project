# GitHub 协作指南

  

> 本项目地址：https://github.com/geyutong985-eng/arduino-project

  

---

  

## 目录

  

- [一、合作逻辑](#一合作逻辑)

- [二、方式一：网页操作](#二方式一网页操作简单但不推荐)

- [三、方式二：终端 + AI](#三方式二终端--ai推荐)

- [四、常见问题](#四常见问题)

  

---

  

> **关于分支**：本指南没有写分支操作。因为我们组文件不多、不同时改同一个文件，分支会增加复杂度。等以后项目大了、有需要了再加。

  

> **关于私密仓库**：本项目是私密仓库，组员需要被邀请才能访问。步骤：

> 1. 老葛在 GitHub 仓库页面 → **Settings** → **Collaborators** → **Add people**

> 2. 输入组员的 GitHub 用户名或邮箱，发送邀请

> 3. 组员会收到邮件，点击接受即可

  

---

  

## 一、合作逻辑

  

### 核心流程

  

```

写代码 → 提交（commit）→ 推送到 GitHub（push）

              ↓

        其他人同步（pull）

```

  

### 基本规则

  

1. **写完就推**：不要等到最后一天才提交，容易出问题

2. **先 pull 再 push**：每天开始前先拉一下最新代码，减少冲突

3. **写清楚改动说明**：commit message 要让人看懂你改了啥

4. **不要动别人的文件**：分工明确，各写各的，减少冲突

5. **常用 `git status`**：随时查看当前状态，知道哪些改了、哪些没改

  

### 常见问题处理

  

- **冲突**：两个人改了同一行。停下来，联系对方商量怎么合并

- **推送失败**：GitHub 上有新的改动，先 `git pull` 拉下来再推送

- **误删文件**：`git checkout -- .` 可以恢复所有文件

  

---

  

## 二、方式一：网页操作（简单但不推荐）

  

> 适合：完全不懂命令行的人，或者只是查看/下载代码

  

### 查看和下载

  

1. 打开 https://github.com/geyutong985-eng/arduino-project

2. 点 **Code → Download ZIP** 下载整个项目

  

### 方式 A：直接在网页上改

  

1. 点进任意文件，点右上角 **编辑图标（铅笔）**

2. 在浏览器里直接改代码

3. 改完后点 **Commit changes**（写清楚改动说明）

4. ⚠️ 注意：这种方式改了 GitHub 上的代码，但你的本地文件夹不会自动更新

  

### 方式 B：下载 → 本地改 → 上传

  

如果你习惯在本地文件夹里写代码，流程是：

  

1. 点 **Code → Download ZIP** 下载代码到本地

2. 解压后在文件夹里改代码

3. 回到 GitHub 页面，点 **Add file → Upload files** 把改好的文件拖进去

4. 点 **Commit changes**

  

这种方式的好处是：本地怎么改 GitHub 就怎么变，不用担心同步问题。

  

### 新建文件

  

1. 点 **Add file → Create new file**

2. 写文件名（如 `sensor.ino`）和内容

3. 点 **Commit changes**

  

---

  

## 三、方式二：终端 + AI（推荐）

  

> 适合：愿意学几个简单命令的人。有 AI 助手的话，几乎不用自己写命令

  

### 为什么推荐终端

  

- 和 AI 助手配合最方便，可以让 AI 帮你写命令

- 操作更灵活，能处理更多情况

- 熟悉之后比网页快很多

  

### 3.1 首次设置（只用做一次）（让AI帮你！！！）

  

**安装 Git（Mac）**

  

打开终端，运行：

  

```bash

brew install git

```

  

**设置身份**

  

```bash

git config --global user.name "你的名字"

git config --global user.email "你的邮箱"

```

  

### 3.2 克隆项目（第一次用）

  

```bash

git clone https://github.com/geyutong985-eng/arduino-project.git

cd arduino-project

```

  

以后直接进这个文件夹操作就行：

  

```bash

cd ~/Desktop/arduino-project

```

  

### 3.3 让 AI 帮你用 Git

  

终端配合 AI 最好用。你不需要记住命令，直接跟 AI 说你要干啥：

  

**示例对话：**

- "帮我把今天的改动提交并推送到 GitHub"

- "有人在 GitHub 上改了代码，帮我同步到本地"

- "我推送失败了，帮我看看什么问题"

  

AI 会帮你写命令，你复制粘贴跑就行。

  

### 3.4 每天写完代码后提交

  

```bash

cd ~/Desktop/arduino-project

git add .

git commit -m "你的改动说明"

git push

```

  

### 3.5 每天开始前同步最新代码

  

```bash

cd ~/Desktop/arduino-project

git pull

```

  

### 3.6 常用命令速查

  

| 场景 | 命令 |

|------|------|

| 查看有哪些改动 | `git status` |

| 把改动加入暂存区 | `git add .` |

| 提交改动 | `git commit -m "说明"` |

| 推送到 GitHub | `git push` |

| 拉取最新代码 | `git pull` |

| 恢复误删的文件 | `git checkout -- .` |

  

---

  

## 四、常见问题

  （让AI帮你！！！）

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