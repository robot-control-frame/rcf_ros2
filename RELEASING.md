# 发布流程

rtcf_ros2 通过 [bloom](https://github.com/ros-infrastructure/bloom) 发布到 ROS 2 Jazzy，
产物落在 [rcf_ros2-release](https://github.com/robot-control-frame/rcf_ros2-release) 仓，
再由 [rosdistro](https://github.com/ros/rosdistro) 里的 `rcf_ros2` 条目驱动 build.ros.org 构建。

```
rcf_ros2 (源码) ──bloom──▶ rcf_ros2-release (打包) ──rosdistro──▶ build.ros.org ──▶ apt
```

## 一次性配置

在 GitHub 仓库 Settings → Secrets and variables → Actions 添加：

| Secret | 说明 |
|---|---|
| `RELEASE_TOKEN` | classic PAT，勾 `repo`；用于推 release 仓和 tag（`GITHUB_TOKEN` 推的 tag 不会触发其他 workflow） |

## 发布一个版本（CI 方式，推荐）

1. Actions → **prepare-release** → Run workflow → 选 `minor` / `patch`。
   它会：生成 CHANGELOG → `catkin_prepare_release` 升版本号 → 提交 → 打 tag `vX.Y.Z` → push。
2. tag push 自动触发 **release** workflow：在 `ros:jazzy` 容器里跑
   `scripts/bloom_release.py`，把 8 个包的 `X.Y.Z-1` 推到 release 仓。
3. 合并 rosdistro PR（把 `jazzy/distribution.yaml` 里 `rcf_ros2.release.version`
   改成 `X.Y.Z-1`）。合入后 build.ros.org 自动构建，约 1 天后可
   `sudo apt install ros-jazzy-rcf`。

## 发布一个版本（本地方式）

```bash
# 1. 升版本号 + CHANGELOG + tag（容器里）
catkin_generate_changelog --all --non-interactive
git add -A && git commit -m "chore: 更新 CHANGELOG"
catkin_prepare_release --bump minor --no-push -y -t v
git push origin main && git push origin v0.7.0
```

`v0.7.0` 推上去后 CI 会自动发布，无需本地再跑 bloom。
网络受限时按 `scripts/bloom_release.py --help` 中说明的本地 staging 镜像方式手动跑：

```bash
git clone --mirror git@github.com:robot-control-frame/rcf_ros2-release.git .rel-staging/release.git
git config --global url."$PWD/.rel-staging/release.git".insteadOf "https://github.com/robot-control-frame/rcf_ros2-release.git"
RELEASE_REPO_URL=https://github.com/robot-control-frame/rcf_ros2-release.git \
UPSTREAM_URL=https://github.com/robot-control-frame/rcf_ros2 \
  python3 scripts/bloom_release.py --version 0.7.0
git -C .rel-staging/release.git push --mirror git@github.com:robot-control-frame/rcf_ros2-release.git
```

## 只修打包、不升版本

同一个版本号重打（产出 `0.7.0-2`）：不升版本号、不新建 tag，直接
Actions → **release** → Run workflow，填同一个版本号并把 `new_track` 取消勾选。

## 常见错误

| 报错 | 原因 |
|---|---|
| `The package(s) in upstream are version '0.6.0', but the version to be released is '0.7.0'` | tag 指向的提交里 `package.xml` 还是旧版本号，即 `catkin_prepare_release` 没跑或没提交 |
| 产物是 `0.6.0-2` 而不是 `0.7.0-1` | 跑 bloom 时没用 `--new-track`，`tracks.yaml` 的版本没更新 |
| 农场报 `no tag release/jazzy/.../0.7.0-1` | release 仓的 tag 没推成功 |
| `Could not resolve rosdep key '...'` | 依赖未在 `package.xml` 里声明为 `depend`/`build_depend` |
