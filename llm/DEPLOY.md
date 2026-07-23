# Robot Command Demo - 部署指南

语音/文字命令 -> LLM 意图解析 -> 结构化机器人指令 -> 命令队列

## 目标部署架构

`robot.mumoiscat.site` -> Cloudflare Tunnel -> 你的电脑本机 `uvicorn` -> SQLite

这条方案不需要 ECS。你的电脑只要能联网，就可以把服务跑在本机，再通过 Cloudflare Tunnel 暴露到公网。

## 1. 需要准备的东西

### 1.1 本机环境

你的电脑上需要：

- Python 3.11 或更高
- 项目代码本体
- 一个可用的 Anthropic 或 OpenAI API Key

### 1.2 Cloudflare 账号和域名托管

要让 `robot.mumoiscat.site` 通过 Tunnel 访问，域名必须托管到 Cloudflare。也就是说，阿里云只负责注册域名，DNS 要切到 Cloudflare。

你需要做两件事：

1. 把 `mumoiscat.site` 这个域名添加到 Cloudflare
2. 在阿里云域名控制台把 nameserver 改成 Cloudflare 提供的那两条

如果你不想改 nameserver，那就不能直接用 `robot.mumoiscat.site`，只能用 Cloudflare 临时给你的 `trycloudflare.com` 地址。

## 2. 先在电脑本地跑起来

### 2.1 创建虚拟环境并安装依赖

在项目根目录执行：

```powershell
cd C:\Users\97239\Desktop\450\robot-command-demo
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

如果你的 PowerShell 不允许激活脚本，先执行：

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

### 2.2 配置 `.env`

复制 [.env.example](.env.example) 为 `.env`，然后填好：

```env
LLM_PROVIDER=anthropic
LLM_API_KEY=你的APIKey
LLM_MODEL=claude-haiku-4-5
DATABASE_PATH=data/queue.db
HOST=127.0.0.1
PORT=8000
```

### 2.3 启动后端

```powershell
uvicorn backend.main:app --host 127.0.0.1 --port 8000
```

本机先验证：

- `http://127.0.0.1:8000/app`
- `http://127.0.0.1:8000/queue`
- `http://127.0.0.1:8000/api/health`

## 3. 配置 Cloudflare Tunnel

### 3.1 安装 cloudflared

在 Windows 上可以用 winget：

```powershell
winget install Cloudflare.cloudflared
```

装好后检查：

```powershell
cloudflared --version
```

### 3.2 登录 Cloudflare

```powershell
& 'C:\Program Files (x86)\cloudflared\cloudflared.exe' tunnel login
```

它会打开浏览器，登录你的 Cloudflare 账号并授权。

### 3.3 创建 Tunnel

```powershell
& 'C:\Program Files (x86)\cloudflared\cloudflared.exe' tunnel create robot-demo
```

这一步会生成一个 Tunnel ID 和凭据文件。

### 3.4 绑定域名到 Tunnel

```powershell
& 'C:\Program Files (x86)\cloudflared\cloudflared.exe' tunnel route dns robot-demo robot.mumoiscat.site
```

这会在 Cloudflare DNS 里创建指向 Tunnel 的记录。

### 3.5 创建配置文件

在 `%USERPROFILE%\\.cloudflared\\config.yml` 写入：

```yaml
tunnel: robot-demo
credentials-file: C:\Users\你的用户名\\.cloudflared\\<tunnel-id>.json

ingress:
  - hostname: robot.mumoiscat.site
    service: http://127.0.0.1:8000
  - service: http_status:404
```

把 `<tunnel-id>.json` 换成 `cloudflared tunnel create` 生成的实际文件名。

### 3.6 启动 Tunnel

```powershell
& 'C:\Program Files (x86)\cloudflared\cloudflared.exe' tunnel run robot-demo
```

如果配置正确，你就可以直接访问：

- `https://robot.mumoiscat.site/app`
- `https://robot.mumoiscat.site/queue`

## 4. 开机自启动

如果你想让电脑开机后自动恢复服务，建议做两个自启动：

1. `uvicorn` 通过任务计划程序或脚本启动
2. `cloudflared tunnel run robot-demo` 通过任务计划程序启动

如果你想省事，也可以先手动启动，确认流程通了再做自启。

## 5. 常见问题

### 域名打不开

先检查：

1. `uvicorn` 是否还在跑
2. `cloudflared tunnel run robot-demo` 是否还在跑
3. Cloudflare 里的 DNS 是否已经生成了 `robot.mumoiscat.site`

### 打开后是 404

通常是 `cloudflared` 的 `ingress` 没指到 `http://127.0.0.1:8000`，或者本机后端没启动。

### 域名还在阿里云 DNS

如果 nameserver 还没切到 Cloudflare，Tunnel 的自定义域名不会生效。这个时候只能先用 Cloudflare 的临时地址，或者把 DNS 迁到 Cloudflare。

## 支持的命令

| 语音/文字输入 | 解析结果 | 机器人服务命令 |
|--------------|---------|---------------|
| "show robot status" | status | status |
| "go to ready pose" | ready | ready |
| "open the gripper" | open_gripper | open |
| "close the gripper" | close_gripper | close |
| "move up a little" | move_up | up |
| "pick up the red block" | invalid | (不执行) |

---

## 项目结构

```
robot-command-demo/
├── backend/
│   ├── main.py           # FastAPI 主程序
│   ├── llm_mapper.py     # LLM 意图解析
│   ├── schema.py         # 数据模型
│   ├── validator.py      # 命令验证
│   ├── queue_store.py    # SQLite 队列存储
│   └── config.py         # 配置加载
├── static/
│   ├── app.html          # 命令输入页面
│   ├── app.js
│   ├── queue.html        # 队列管理页面
│   └── queue.js
├── data/
│   └── queue.db          # SQLite 数据库（自动生成）
├── .env                  # API Key 配置（不要提交）
├── .env.example          # 配置模板
├── requirements.txt      # Python 依赖
├── ngrok.exe             # ngrok 可执行文件
└── DEPLOY.md             # 本文档
```

---

## API 接口

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | /api/text-command | 提交文字命令 |
| GET | /api/queue | 获取所有队列项 |
| GET | /api/queue/current-posted | 获取当前 posted 项 |
| POST | /api/queue/{id}/approve | 批准/发布命令 |
| POST | /api/queue/{id}/reject | 拒绝命令 |
| DELETE | /api/queue/{id} | 删除命令 |
| POST | /api/queue/clear | 清空队列 |
| GET | /download | 下载当前 posted 项的 JSON |

---

## 常见问题

### LLM 返回 404 错误

旧模型已退役，请在 `.env` 中使用新模型名：
- `claude-haiku-4-5`（推荐，便宜快速）
- `claude-sonnet-4-5`（更智能）
- `claude-opus-4-8`（最强）

### Cloudflare Tunnel 绑定失败

一般是这几种原因：

1. `mumoiscat.site` 还没有切到 Cloudflare 托管
2. `cloudflared tunnel login` 没有完成授权
3. `cloudflared tunnel route dns` 没有成功创建记录
4. 本机 `uvicorn` 没有在 `127.0.0.1:8000` 监听

### 服务器启动后立即停止

检查是否有其他进程占用 8000 端口：
```powershell
netstat -ano | findstr :8000
```

---

## 本地运行（可选）

如果你只是想先本地验证，可以参考：

1. 创建虚拟环境
2. 安装依赖
3. 配置 `.env`
4. 启动 `uvicorn backend.main:app --host 127.0.0.1 --port 8000`

然后访问 `http://127.0.0.1:8000/app`。
