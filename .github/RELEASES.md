# Releases 使用指南

## 什么是 Releases？

GitHub Releases 是项目的版本发布功能，可以：
- 为每个版本附加编译好的固件文件（.elf, .bin, .hex）
- 自动生成版本说明（changelog）
- 提供稳定的下载链接

## 如何创建 Release？

### 方法 1：通过 Git Tag（推荐，自动构建）

1. **创建并推送 tag**：
   ```bash
   git tag -a v1.0.0 -m "Release version 1.0.0"
   git push origin v1.0.0
   ```

2. **GitHub Actions 会自动**：
   - 编译固件
   - 生成 Release
   - 上传固件文件（.elf, .bin, .hex）

3. **在 GitHub 网页查看**：
   - 访问 `https://github.com/Kitjesen/Vectorfoc/releases`
   - 可以看到新创建的 Release

### 方法 2：手动创建（GitHub 网页）

1. 访问 `https://github.com/Kitjesen/Vectorfoc/releases`
2. 点击 "Draft a new release"
3. 填写：
   - **Tag**: `v1.0.0`（需要先创建 tag）
   - **Title**: `VectorFOC v1.0.0`
   - **Description**: 版本说明
4. 手动上传编译好的固件文件
5. 点击 "Publish release"

## Tag 命名规范

- **正式版本**: `v1.0.0`, `v1.1.0`, `v2.0.0`
- **预发布版本**: `v1.0.0-alpha`, `v1.0.0-beta`, `v1.0.0-rc1`
- **开发版本**: `v1.0.0-dev`

预发布版本会自动标记为 "Pre-release"。

## 固件文件说明

| 文件 | 用途 | 工具 |
|------|------|------|
| `.elf` | 完整调试信息，用于调试 | STM32CubeIDE, Keil |
| `.bin` | 二进制镜像，用于烧录 | STM32CubeProgrammer |
| `.hex` | Intel HEX 格式，通用烧录 | 各种烧录工具 |

## CI/CD 流程

```
推送代码 → CI 自动测试 → 通过 ✅
    ↓
创建 Tag → Release Workflow → 自动构建 → 发布 Release
```

## 查看构建历史

- **CI 构建**: `https://github.com/Kitjesen/Vectorfoc/actions`
- **Releases**: `https://github.com/Kitjesen/Vectorfoc/releases`
- **构建产物**: 在 Actions 页面点击每次构建，可以下载 `firmware-binaries` artifact
