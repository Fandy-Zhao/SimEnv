# Issue: 克隆 SimEnv Gitee 仓库并连接 GitHub 远程仓库

## 任务目标
将 Gitee 上的 SimEnv 仓库连接 GitHub 远程仓库，方便后续把治理后的开发工作同步到 GitHub。

## 修改范围
- git remote 配置: 新增 `github` remote
- `AGENTS.md`: 写入项目级分支覆盖规则 (维护类使用 `zzf/`)
- `PROJECT_STATE.md`, `CHANGELOG.md`, `docs/module_status.md`: 更新治理文档
- 操作记录: `experiments/runs/0704_connect-github-remote/`

## 非范围
- 不改业务代码
- 不 push 到 GitHub
- 不 merge 到 main/master
- 不删除文件
- 不覆盖已有 remote

## 验收标准
- [x] 本地存在 SimEnv git 仓库 (已存在于 `/home/zzf/search_ws/SimEnv`)
- [x] `origin` 保持指向 Gitee (`https://gitee.com/guoyulun/SimEnv.git`)
- [x] 新增 `github` remote 指向 `git@github.com:zzf/SimEnv.git`
- [x] governance 骨架存在
- [x] `AGENTS.md` 记录维护类分支使用 `zzf/MMDD-short-name`
- [x] 不再把维护类任务分支写成 `chore/MMDD-short-name`
- [x] 有 commit 记录本次治理初始化或远程配置说明
- [x] 没有执行 push

## 风险点
- GitHub 远程仓库可能为空或已有历史
- 后续 push 前需要确认目标分支和是否保留完整历史
- 分支规则已做项目级覆盖，后续 Codex 需要遵守 `zzf/` 维护分支规则
